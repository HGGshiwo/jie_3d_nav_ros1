import * as THREE from 'three';
import { initScene } from './scene.js';
import { LayerManager } from './layer.js';

// ---- 0. 编辑状态与地图版本控制 ----
let isDirty = false;
let isPreblockedDirty = false;
let serverPreblockedVersion = 0;
let serverOccupiedVersion = 0;

// 全局 DOM 节点缓存，防止 TDZ（暂存死区）及高频 DOM 查询导致的卡顿
const statusEl = document.getElementById('status');
const rootPathInput = document.getElementById('root-path');
const mapNameInput = document.getElementById('map-name');
const brushSizeInput = document.getElementById('brush-size');
const editZInput = document.getElementById('edit-z');
const editLayerSelect = document.getElementById('edit-layer');
const debugPanelDiv = document.getElementById('debug-panel');
const btnCopyDebug = document.getElementById('btn-copy-debug');
const btnFocusRobot = document.getElementById('btn-focus-robot');
const rosStatusEl = document.getElementById('ros-status');

// 全局参数缓存
let currentTool = 'view';
let currentLayer = 'occupied';
let brushSize = 1;
let zHeight = 0.0;
let latestDebugData = null;

// 从 HTML 默认状态初始化缓存参数
if (brushSizeInput) brushSize = parseInt(brushSizeInput.value) || 1;
if (editZInput) zHeight = parseFloat(editZInput.value) || 0.0;
if (editLayerSelect) currentLayer = editLayerSelect.value;
const checkedTool = document.querySelector('input[name="tool"]:checked');
if (checkedTool) currentTool = checkedTool.value;

// ---- 1. 场景初始化 ----
const container = document.getElementById('canvas-container');
const { scene, camera, renderer, controls, editPlane } = initScene(container);
renderer.domElement.addEventListener('contextmenu', e => e.preventDefault());

// ---- 2. 高性能体素管理系统 ----
// 容器 (默认分辨率初始化为 0.4)
let layers = {
    occupied: new LayerManager(scene, "occupied", [0.4, 0.4, 0.4]),
    preblocked: new LayerManager(scene, "preblocked", [0.4, 0.4, 0.4]),
    traversable: new LayerManager(scene, "traversable", [0.4, 0.4, 0.4]),
    risk_cost: new LayerManager(scene, "risk_cost", [0.4, 0.4, 0.4]),
    local_octomap: new LayerManager(scene, "local_octomap", [0.2, 0.2, 0.2]),
    fused_octomap: new LayerManager(scene, "fused_octomap", [0.2, 0.2, 0.2]),
    emergency_stop_free: new LayerManager(scene, "emergency_stop_free", [0.06, 0.06, 0.06]),
    emergency_stop_occupied: new LayerManager(scene, "emergency_stop_occupied", [0.10, 0.10, 0.10])
};
layers.traversable.mesh.visible = false;
layers.risk_cost.mesh.visible = false;
layers.local_octomap.mesh.visible = false;
layers.fused_octomap.mesh.visible = false;
layers.emergency_stop_free.mesh.visible = false;
layers.emergency_stop_occupied.mesh.visible = false;

// 检查并返回当前已勾选需要传输的图层列表
function getActiveRequestedLayers() {
    const requested = [];
    if (document.getElementById('show-occupied')?.checked) requested.push('occupied');
    if (document.getElementById('show-preblocked')?.checked) requested.push('preblocked');
    if (document.getElementById('show-traversable')?.checked) requested.push('traversable');
    if (document.getElementById('show-risk-cost')?.checked) requested.push('risk_cost');
    if (document.getElementById('show-octomap-local')?.checked) requested.push('local_octomap');
    if (document.getElementById('show-octomap-fused')?.checked) requested.push('fused_octomap');
    if (document.getElementById('show-emergency-stop')?.checked) {
        requested.push('emergency_stop_free');
        requested.push('emergency_stop_occupied');
    }
    return requested;
}

// 监听复选框，切换图层显隐
document.getElementById('show-occupied').addEventListener('change', (e) => {
    layers.occupied.mesh.visible = e.target.checked;
});
document.getElementById('show-preblocked').addEventListener('change', (e) => {
    layers.preblocked.mesh.visible = e.target.checked;
});
document.getElementById('show-traversable').addEventListener('change', (e) => {
    layers.traversable.mesh.visible = e.target.checked;
});
document.getElementById('show-risk-cost').addEventListener('change', (e) => {
    layers.risk_cost.mesh.visible = e.target.checked;
});
document.getElementById('show-octomap-local')?.addEventListener('change', (e) => {
    layers.local_octomap.mesh.visible = e.target.checked;
    if (!e.target.checked) layers.local_octomap.loadFromArray([], layers.local_octomap.scale);
});
document.getElementById('show-octomap-fused')?.addEventListener('change', (e) => {
    layers.fused_octomap.mesh.visible = e.target.checked;
    if (!e.target.checked) layers.fused_octomap.loadFromArray([], layers.fused_octomap.scale);
});
document.getElementById('show-emergency-stop')?.addEventListener('change', (e) => {
    layers.emergency_stop_free.mesh.visible = e.target.checked;
    layers.emergency_stop_occupied.mesh.visible = e.target.checked;
    if (!e.target.checked) {
        layers.emergency_stop_free.loadFromArray([], layers.emergency_stop_free.scale);
        layers.emergency_stop_occupied.loadFromArray([], layers.emergency_stop_occupied.scale);
    }
});
const showRobotEl = document.getElementById('show-robot');
if (showRobotEl) {
    showRobotEl.addEventListener('change', (e) => {
        if (dogMeshGroup) dogMeshGroup.visible = e.target.checked;
    });
}

// ---- 3. 编辑交互逻辑 ----
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
let isPainting = false;

// 游标提示框（单位尺寸几何体，动态应用 scale）
const cursorGeo = new THREE.BoxGeometry(1.0, 1.0, 1.0);
const cursorMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
const cursor = new THREE.Mesh(cursorGeo, cursorMat);
scene.add(cursor);

// 重构后的参数获取函数，直接读取内存缓存，耗时为 0ms
function getBrushParams() {
    return {
        tool: currentTool,
        layer: currentLayer,
        size: brushSize,
        zHeight: zHeight
    };
}

// 绑定事件以更新参数缓存
document.querySelectorAll('input[name="tool"]').forEach(radio => {
    radio.addEventListener('change', (e) => {
        currentTool = e.target.value;
        if (debugPanelDiv) {
            if (currentTool === 'debug') {
                debugPanelDiv.style.display = 'block';
            } else {
                debugPanelDiv.style.display = 'none';
            }
        }
    });
});

if (editLayerSelect) {
    editLayerSelect.addEventListener('change', (e) => {
        currentLayer = e.target.value;
    });
}

if (brushSizeInput) {
    brushSizeInput.addEventListener('input', (e) => {
        brushSize = parseInt(e.target.value) || 1;
    });
}

if (editZInput) {
    editZInput.addEventListener('input', (e) => {
        zHeight = parseFloat(e.target.value) || 0.0;
        if (editPlane) {
            editPlane.position.z = zHeight;
        }
    });
    editZInput.addEventListener('change', (e) => {
        zHeight = parseFloat(e.target.value) || 0.0;
        if (editPlane) {
            editPlane.position.z = zHeight;
        }
    });
}

let activeHit = null; // 存储当前鼠标射线击中的目标

function getInteractionTarget() {
    const params = getBrushParams();
    const activeLayerName = params.layer;
    const targetLayer = layers[activeLayerName];
    const scale = targetLayer.scale;

    raycaster.setFromCamera(mouse, camera);

    // 1. 尝试与场景中所有显示的体素求交
    const meshesToIntersect = [];
    Object.keys(layers).forEach(name => {
        if (layers[name] && layers[name].mesh && layers[name].mesh.visible) {
            meshesToIntersect.push(layers[name].mesh);
        }
    });

    const voxelIntersects = raycaster.intersectObjects(meshesToIntersect);
    
    if (params.tool === 'eraser') {
        console.log("【橡皮擦调试】可求交网格数:", meshesToIntersect.length, "；射线交点个数:", voxelIntersects.length);
    }
    
    if (voxelIntersects.length > 0) {
        const hit = voxelIntersects[0];
        const hitMesh = hit.object;
        const instanceId = hit.instanceId;
        
        let hitLayerName = null;
        Object.keys(layers).forEach(name => {
            if (layers[name] && layers[name].mesh === hitMesh) {
                hitLayerName = name;
            }
        });
        
        if (params.tool === 'eraser') {
            console.log("【橡皮擦调试】击中网格图层:", hitLayerName, "；instanceId:", instanceId);
        }
        
        if (hitLayerName) {
            const voxel = layers[hitLayerName].getVoxelByInstanceId(instanceId);
            if (params.tool === 'eraser') {
                console.log("【橡皮擦调试】提取对应的体素数据:", voxel);
            }
            if (voxel) {
                const normal = hit.face.normal.clone();
                const vScale = voxel.scale || layers[hitLayerName].scale;
                return {
                    type: 'voxel',
                    layerName: hitLayerName,
                    voxel: voxel,
                    normal: normal,
                    scale: vScale
                };
            }
        }
    }

    // 2. 如果没有射中体素，且工具不是橡皮擦，则尝试与参考平面求交
    if (params.tool !== 'eraser') {
        const planeIntersects = raycaster.intersectObject(editPlane);
        if (planeIntersects.length > 0) {
            const hit = planeIntersects[0];
            // 限制交点到相机的距离（防止低角度射线滑移到无穷远处导致包围盒爆炸）
            if (hit.distance < 80) {
                const cx = Math.round(hit.point.x / scale[0]) * scale[0];
                const cy = Math.round(hit.point.y / scale[1]) * scale[1];
                const cz = params.zHeight;
                // 限制在距原点 50m 的有效区域内
                if (Math.abs(cx) < 50 && Math.abs(cy) < 50) {
                    return {
                        type: 'plane',
                        layerName: activeLayerName,
                        position: { x: cx, y: cy, z: cz },
                        scale: scale
                    };
                }
            }
        }
    }

    return null;
}

function executePaint() {
    if (!activeHit) return;
    
    const params = getBrushParams();
    const targetLayer = layers[params.layer];
    const scale = targetLayer.scale;
    
    if (params.tool === 'eraser') {
        if (activeHit.type === 'voxel') {
            const cx = activeHit.voxel.x;
            const cy = activeHit.voxel.y;
            const cz = activeHit.voxel.z;
            const half = (params.size - 1) / 2;
            
            for (let dx = -half; dx <= half; dx++) {
                for (let dy = -half; dy <= half; dy++) {
                    const px = cx + dx * activeHit.scale[0];
                    const py = cy + dy * activeHit.scale[1];
                    // 同时擦除占据层（橙色）和禁行层（红色），避免单独残留导致C++端根据另一层重新计算恢复
                    const removedOcc = layers.occupied.removeVoxel(px, py, cz);
                    const removedPre = layers.preblocked.removeVoxel(px, py, cz);
                    if (removedOcc || removedPre) {
                        isDirty = true;
                        if (removedPre) {
                            isPreblockedDirty = true;
                        }
                    }
                }
            }
            activeHit = getInteractionTarget();
            updateCursorVisual();
        }
    } else if (params.tool === 'brush') {
        let cx, cy, cz;
        if (activeHit.type === 'voxel') {
            cx = activeHit.voxel.x + activeHit.normal.x * activeHit.scale[0];
            cy = activeHit.voxel.y + activeHit.normal.y * activeHit.scale[1];
            cz = activeHit.voxel.z + activeHit.normal.z * activeHit.scale[2];
        } else if (activeHit.type === 'plane') {
            cx = activeHit.position.x;
            cy = activeHit.position.y;
            cz = activeHit.position.z;
        }
        
        const half = (params.size - 1) / 2;
        for (let dx = -half; dx <= half; dx++) {
            for (let dy = -half; dy <= half; dy++) {
                const px = cx + dx * scale[0];
                const py = cy + dy * scale[1];
                if (targetLayer.addVoxel(px, py, cz)) {
                    isDirty = true;
                    if (params.layer === 'preblocked') {
                        isPreblockedDirty = true;
                    }
                }
            }
        }
        activeHit = getInteractionTarget();
        updateCursorVisual();
    }
}

function updateCursorVisual() {
    const params = getBrushParams();
    if (params.tool === 'view' || isSpacePressed || !activeHit) {
        cursor.visible = false;
        return;
    }

    cursor.visible = true;
    if (params.tool === 'start' || params.tool === 'goal') {
        cursor.material.color.setHex(params.tool === 'start' ? 0x00ff00 : 0xff0000);
        if (activeHit.type === 'voxel') {
            cursor.position.set(activeHit.voxel.x, activeHit.voxel.y, activeHit.voxel.z + activeHit.scale[2]/2 + 0.3);
        } else if (activeHit.type === 'plane') {
            cursor.position.set(activeHit.position.x, activeHit.position.y, activeHit.position.z + 0.3);
        }
        cursor.scale.set(0.3, 0.3, 0.3); // 预览圆球（以小方块尺寸代替）
        return;
    }

    if (params.tool === 'debug' || params.tool === 'debug_air') {
        cursor.material.color.setHex(params.tool === 'debug' ? 0x2196F3 : 0x00BCD4); // 蓝色表示已有体素，青色表示空网格
        if (activeHit.type === 'voxel') {
            let newX, newY, newZ;
            if (params.tool === 'debug') {
                newX = activeHit.voxel.x;
                newY = activeHit.voxel.y;
                newZ = activeHit.voxel.z;
            } else {
                newX = activeHit.voxel.x + activeHit.normal.x * activeHit.scale[0];
                newY = activeHit.voxel.y + activeHit.normal.y * activeHit.scale[1];
                newZ = activeHit.voxel.z + activeHit.normal.z * activeHit.scale[2];
            }
            cursor.position.set(newX, newY, newZ);
            cursor.scale.set(activeHit.scale[0] * 0.98, activeHit.scale[1] * 0.98, activeHit.scale[2] * 1.02);
        } else if (activeHit.type === 'plane') {
            cursor.position.set(activeHit.position.x, activeHit.position.y, activeHit.position.z);
            cursor.scale.set(activeHit.scale[0] * 0.98, activeHit.scale[1] * 0.98, activeHit.scale[2] * 1.02);
        }
        return;
    }

    if (params.tool === 'eraser') {
        if (activeHit.type === 'voxel') {
            cursor.position.set(activeHit.voxel.x, activeHit.voxel.y, activeHit.voxel.z);
            cursor.scale.set(activeHit.scale[0] * 1.02 * params.size, activeHit.scale[1] * 1.02 * params.size, activeHit.scale[2] * 1.05);
            cursor.material.color.setHex(0xff0000); // 红色高亮表示即将删除该体素
        } else {
            cursor.visible = false;
        }
    } else {
        cursor.material.color.setHex(0xffffff); // 白色高亮表示即将放置
        const scale = layers[params.layer].scale;
        if (activeHit.type === 'voxel') {
            const newX = activeHit.voxel.x + activeHit.normal.x * activeHit.scale[0];
            const newY = activeHit.voxel.y + activeHit.normal.y * activeHit.scale[1];
            const newZ = activeHit.voxel.z + activeHit.normal.z * activeHit.scale[2];
            cursor.position.set(newX, newY, newZ);
            cursor.scale.set(scale[0] * 0.98 * params.size, scale[1] * 0.98 * params.size, scale[2] * 1.02);
        } else if (activeHit.type === 'plane') {
            cursor.position.set(activeHit.position.x, activeHit.position.y, activeHit.position.z);
            cursor.scale.set(scale[0] * 0.98 * params.size, scale[1] * 0.98 * params.size, scale[2] * 1.02);
        }
    }
}

// ---- 3.1 键盘控制与快捷切视角 ----
const keysPressed = {};
let isSpacePressed = false;

// 第一人称（MC视角）拖动控制变量
let cameraPitch = 0;
let cameraYaw = 0;
let isFPDragging = false;
let prevMouseX = 0;
let prevMouseY = 0;

function syncPitchYawFromCamera() {
    const dir = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
    cameraYaw = Math.atan2(dir.y, dir.x);
    const xyLen = Math.sqrt(dir.x * dir.x + dir.y * dir.y);
    cameraPitch = Math.atan2(dir.z, xyLen);
}

function isInputFocused() {
    const el = document.activeElement;
    return el && (el.tagName === 'INPUT' || el.tagName === 'SELECT' || el.tagName === 'TEXTAREA');
}

window.addEventListener('keydown', (e) => {
    if (isInputFocused()) return;
    keysPressed[e.code] = true;
    
    if (e.code === 'Space') {
        isSpacePressed = true;
        document.body.style.cursor = 'grab';
        cursor.visible = false;
    }
});

window.addEventListener('keyup', (e) => {
    keysPressed[e.code] = false;
    
    if (e.code === 'Space') {
        isSpacePressed = false;
        document.body.style.cursor = 'default';
        const params = getBrushParams();
        if (params.tool !== 'view') {
            cursor.visible = true;
        }
    }
});

// 起终点和路径可视化对象
let startMesh = null;
let goalMesh = null;
let pathLine = null;

function updateStartVisual(x, y, z) {
    if (startMesh) scene.remove(startMesh);
    const geo = new THREE.SphereGeometry(0.3, 16, 16);
    const mat = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    startMesh = new THREE.Mesh(geo, mat);
    startMesh.position.set(x, y, z);
    scene.add(startMesh);
}

function updateGoalVisual(x, y, z) {
    if (goalMesh) scene.remove(goalMesh);
    const geo = new THREE.SphereGeometry(0.3, 16, 16);
    const mat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    goalMesh = new THREE.Mesh(geo, mat);
    goalMesh.position.set(x, y, z);
    scene.add(goalMesh);
}

function updatePathVisual(points) {
    if (pathLine) scene.remove(pathLine);
    if (!points || points.length === 0) return;
    
    const threePoints = points.map(pt => new THREE.Vector3(pt[0], pt[1], pt[2]));
    const geometry = new THREE.BufferGeometry().setFromPoints(threePoints);
    const material = new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 3 }); // 青色路径
    pathLine = new THREE.Line(geometry, material);
    scene.add(pathLine);
}

async function setStartPoint(x, y, z) {
    updateStartVisual(x, y, z);
    statusEl.innerText = `正在设置起点: [${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}]`;
    try {
        const res = await fetch('/api/set_start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, z })
        });
        if (res.ok) {
            const data = await res.json();
            statusEl.innerText = data.message;
        }
    } catch (err) {
        statusEl.innerText = `设置起点失败: ${err.message}`;
    }
}

async function setGoalPoint(x, y, z) {
    updateGoalVisual(x, y, z);
    statusEl.innerText = `正在设置终点: [${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}]`;
    try {
        const res = await fetch('/api/set_goal', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, z })
        });
        if (res.ok) {
            const data = await res.json();
            statusEl.innerText = data.message;
        }
    } catch (err) {
        statusEl.innerText = `设置终点失败: ${err.message}`;
    }
}

// 机器狗 3D 可视化模型与轮询更新
let dogMeshGroup = null;

function updateRobotVisual(position, orientation) {
    if (!dogMeshGroup) {
        dogMeshGroup = new THREE.Group();

        // 狗主体 (科技蓝长方体)
        const bodyGeo = new THREE.BoxGeometry(0.5, 0.35, 0.25);
        const bodyMat = new THREE.MeshStandardMaterial({ color: 0x3b82f6, metalness: 0.3, roughness: 0.4 });
        const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
        bodyMesh.position.z = 0.125;
        dogMeshGroup.add(bodyMesh);

        // 狗头/朝向指示器 (指向 +X 轴的红色圆锥)
        const headGeo = new THREE.ConeGeometry(0.18, 0.4, 16);
        headGeo.rotateZ(-Math.PI / 2); // 让顶点指向 +X 轴
        const headMat = new THREE.MeshStandardMaterial({ color: 0xef4444, metalness: 0.2, roughness: 0.3 });
        const headMesh = new THREE.Mesh(headGeo, headMat);
        headMesh.position.set(0.35, 0, 0.15);
        dogMeshGroup.add(headMesh);

        // 底部定位光圈
        const ringGeo = new THREE.RingGeometry(0.3, 0.42, 32);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x60a5fa, side: THREE.DoubleSide, transparent: true, opacity: 0.8 });
        const ringMesh = new THREE.Mesh(ringGeo, ringMat);
        ringMesh.position.z = 0.01;
        dogMeshGroup.add(ringMesh);

        const showRobotEl = document.getElementById('show-robot');
        if (showRobotEl) {
            dogMeshGroup.visible = showRobotEl.checked;
        }
        scene.add(dogMeshGroup);
    }

    dogMeshGroup.position.set(position.x, position.y, position.z);
    if (orientation) {
        dogMeshGroup.quaternion.set(orientation.x, orientation.y, orientation.z, orientation.w);
    }
}

// 轮询获取机器狗实时定位信息（每 200ms 高频更新）
setInterval(async () => {
    try {
        const res = await fetch('/api/get_robot_pose');
        if (res.ok) {
            const data = await res.json();
            if (data.has_pose && data.position) {
                updateRobotVisual(data.position, data.orientation);
            }
        }
    } catch (err) {
        // 忽略网络抖动
    }
}, 200);

// 显示与隐藏全局悬浮加载提示框
function showLoading(msg = "⏳ 正在加载地图数据，请稍候...") {
    const indicator = document.getElementById('loading-indicator');
    const msgEl = document.getElementById('loading-msg');
    if (indicator && msgEl) {
        msgEl.innerText = msg;
        indicator.style.display = 'block';
        indicator.style.opacity = '1';
    }
}

function hideLoading() {
    const indicator = document.getElementById('loading-indicator');
    if (indicator) {
        indicator.style.opacity = '0';
        setTimeout(() => { indicator.style.display = 'none'; }, 300);
    }
}

// 视角一键对准机器狗位置（第一人称/追随视角）
function focusOnRobot() {
    if (!dogMeshGroup || !dogMeshGroup.position) {
        if (statusEl) statusEl.innerText = '无法对准视角: 未收到机器狗当前定位';
        return;
    }

    const pos = dogMeshGroup.position.clone();
    const quat = dogMeshGroup.quaternion.clone();

    // 根据四元数计算机器狗前向 (+X 轴) 与上向 (+Z 轴) 向量
    const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(quat);
    const up = new THREE.Vector3(0, 0, 1);

    // 将相机位置设定在机器狗后方 2.0m、上方 1.2m 处（类似于第一人称/第三人称追随视角）
    const camPos = pos.clone()
        .sub(forward.clone().multiplyScalar(2.0))
        .add(up.clone().multiplyScalar(1.2));

    // 将控制焦点（Look-At 点）设定在机器狗前方 2.5m 处
    const targetPos = pos.clone().add(forward.clone().multiplyScalar(2.5)).addScaledVector(up, 0.3);

    // 赋值相机与 OrbitControls
    camera.position.copy(camPos);
    controls.target.copy(targetPos);
    controls.update();

    if (statusEl) statusEl.innerText = `已切换至第一人称追随视角: [${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)}]`;
}

if (btnFocusRobot) {
    btnFocusRobot.addEventListener('click', focusOnRobot);
}

// 轮询获取 ROS 规划状态消息 (500ms 刷新)
setInterval(async () => {
    try {
        const res = await fetch('/api/get_status_text');
        if (res.ok) {
            const data = await res.json();
            if (data.status_text && rosStatusEl) {
                rosStatusEl.innerText = `ROS 状态: ${data.status_text}`;
            }
        }
    } catch (err) {
        // 忽略网络抖动
    }
}, 500);

// 轮询获取 ROS 规划路径数据 (500ms 刷新)
setInterval(async () => {
    try {
        const res = await fetch('/api/get_path');
        if (res.ok) {
            const data = await res.json();
            if (data.path) {
                updatePathVisual(data.path);
            }
        }
    } catch (err) {
        // 忽略网络抖动
    }
}, 500);

// 动态图层轮询（仅当在前端勾选图层时，才以 300ms 频率向后端请求对应图层数据；未勾选则完全不请求）
setInterval(async () => {
    const activeLayers = getActiveRequestedLayers();
    if (activeLayers.length === 0) return;
    
    try {
        const queryStr = `?layers=${activeLayers.join(',')}`;
        const res = await fetch(`/api/get_current_map${queryStr}`);
        if (res.ok) {
            const data = await res.json();
            if (data.layers) {
                if (data.layers.local_octomap && document.getElementById('show-octomap-local')?.checked) {
                    layers.local_octomap.loadFromArray(data.layers.local_octomap.points, data.layers.local_octomap.scale, null, data.layers.local_octomap.groups);
                }
                if (data.layers.fused_octomap && document.getElementById('show-octomap-fused')?.checked) {
                    layers.fused_octomap.loadFromArray(data.layers.fused_octomap.points, data.layers.fused_octomap.scale, null, data.layers.fused_octomap.groups);
                }
                if (document.getElementById('show-emergency-stop')?.checked) {
                    if (data.layers.emergency_stop_free) {
                        layers.emergency_stop_free.loadFromArray(data.layers.emergency_stop_free.points, data.layers.emergency_stop_free.scale, null, data.layers.emergency_stop_free.groups);
                    }
                    if (data.layers.emergency_stop_occupied) {
                        layers.emergency_stop_occupied.loadFromArray(data.layers.emergency_stop_occupied.points, data.layers.emergency_stop_occupied.scale, null, data.layers.emergency_stop_occupied.groups);
                    }
                }
                if (data.layers.traversable && document.getElementById('show-traversable')?.checked) {
                    layers.traversable.loadFromArray(data.layers.traversable.points, data.layers.traversable.scale, null, data.layers.traversable.groups);
                }
                if (data.layers.risk_cost && document.getElementById('show-risk-cost')?.checked) {
                    layers.risk_cost.loadFromArray(data.layers.risk_cost.points, data.layers.risk_cost.scale, data.layers.risk_cost.intensities, data.layers.risk_cost.groups);
                }
                if (!isDirty) {
                    if (data.layers.occupied && document.getElementById('show-occupied')?.checked) {
                        layers.occupied.loadFromArray(data.layers.occupied.points, data.layers.occupied.scale, null, data.layers.occupied.groups);
                    }
                    if (data.layers.preblocked && document.getElementById('show-preblocked')?.checked) {
                        layers.preblocked.loadFromArray(data.layers.preblocked.points, data.layers.preblocked.scale, null, data.layers.preblocked.groups);
                    }
                }
            }
        }
    } catch (err) {
        // 忽略网络抖动
    }
}, 300);

// 页面加载时自动获取先前缓存的地图配置或默认配置并加载，避免手动重新输入
async function initDefaultMap() {
    const cachedRoot = localStorage.getItem('map_root_path');
    const cachedName = localStorage.getItem('map_name');
    
    if (cachedRoot && cachedName) {
        document.getElementById('root-path').value = cachedRoot;
        document.getElementById('map-name').value = cachedName;
        console.log("【本地缓存】成功恢复上次配置路径:", cachedRoot, cachedName);
    } else {
        try {
            const res = await fetch('/api/default_map');
            if (res.ok) {
                const data = await res.json();
                if (data.root_path && data.map_name) {
                    document.getElementById('root-path').value = data.root_path;
                    document.getElementById('map-name').value = data.map_name;
                }
            }
        } catch (err) {
            console.error("Failed to load default map config:", err);
        }
    }
    
    // 拉取当前 ROS 中活跃的地图数据
    console.log("【网页初始化】拉取当前 ROS 活跃的地图数据...");
    showLoading("⏳ 正在初始化载入 3D 地图...");
    await reloadMapFromServer(true);
    requestAnimationFrame(() => {
        requestAnimationFrame(() => {
            hideLoading();
        });
    });
}
initDefaultMap();

let lastMoveTime = 0;
window.addEventListener('pointermove', (event) => {
    if (isFPDragging) {
        const deltaX = event.clientX - prevMouseX;
        const deltaY = event.clientY - prevMouseY;
        prevMouseX = event.clientX;
        prevMouseY = event.clientY;

        const sensitivity = 0.0035;
        cameraYaw -= deltaX * sensitivity;
        cameraPitch -= deltaY * sensitivity;

        // Clamp pitch to avoid upside-down view
        const maxPitch = Math.PI / 2 - 0.05;
        cameraPitch = Math.max(-maxPitch, Math.min(maxPitch, cameraPitch));

        const targetOffset = new THREE.Vector3(
            Math.cos(cameraYaw) * Math.cos(cameraPitch),
            Math.sin(cameraYaw) * Math.cos(cameraPitch),
            Math.sin(cameraPitch)
        );
        
        const targetPoint = camera.position.clone().addScaledVector(targetOffset, 20);
        camera.lookAt(targetPoint);
        controls.target.copy(targetPoint);
        return;
    }

    const now = performance.now();
    if (now - lastMoveTime < 30) return; // 节流控制，最多 33 FPS，防止卡顿
    lastMoveTime = now;

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
    
    if (event.clientX < 330) {
        cursor.visible = false;
        activeHit = null;
        return;
    }

    // 性能优化点：若处于拖动视角模式或按住空格，直接跳过耗时的三维射线碰撞检测
    const params = getBrushParams();
    if (params.tool === 'view' || isSpacePressed) {
        cursor.visible = false;
        activeHit = null;
        return;
    }
    
    activeHit = getInteractionTarget();
    updateCursorVisual();

    if (isPainting) {
        executePaint();
    }
});

async function queryCellDebugInfo(x, y, z, layerName = '') {
    statusEl.innerText = `正在查询网格诊断信息...`;
    try {
        const res = await fetch('/api/debug_cell', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, z, layer_name: layerName })
        });
        if (!res.ok) throw new Error((await res.json()).detail || "查询失败");
        const data = await res.json();
        if (data.status === 'success') {
            statusEl.innerText = "诊断信息查询成功";
            document.getElementById('debug-grid-coord').innerText = `[${data.grid_x}, ${data.grid_y}, ${data.grid_z}]`;
            document.getElementById('debug-world-coord').innerText = `[${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}]`;
            
            // 占据状态
            if (data.is_unknown) {
                document.getElementById('debug-occupied').innerText = "未知 (Unknown)";
                document.getElementById('debug-occupied').style.color = "#FF9800";
            } else {
                document.getElementById('debug-occupied').innerText = data.is_occupied ? "是 (Occupied)" : "否 (Free)";
                document.getElementById('debug-occupied').style.color = data.is_occupied ? "#f44336" : "#4CAF50";
            }
            
            // 地面支撑
            document.getElementById('debug-ground-support').innerText = data.has_ground_support ? "是 (Supported)" : "否 (No Support)";
            document.getElementById('debug-ground-support').style.color = data.has_ground_support ? "#4CAF50" : "#f44336";
            
            // 禁行状态与原因
            document.getElementById('debug-preblocked').innerText = data.is_preblocked ? "是 (Preblocked)" : "否";
            document.getElementById('debug-preblocked').style.color = data.is_preblocked ? "#f44336" : "#4CAF50";
            
            const reasonMap = {
                "none": "无",
                "manual": "手动绘制禁行",
                "step_or_obstacle_edge": "台阶或障碍物边缘",
                "cliff_or_suspended": "悬空或悬崖边缘"
            };
            document.getElementById('debug-preblocked-reason').innerText = reasonMap[data.preblocked_reason] || data.preblocked_reason;
            
            // 碰撞与通路阻断
            document.getElementById('debug-horizontal-col').innerText = data.has_horizontal_collision ? "是 (Collision)" : "无";
            document.getElementById('debug-horizontal-col').style.color = data.has_horizontal_collision ? "#f44336" : "#4CAF50";
            
            document.getElementById('debug-vertical-col').innerText = data.has_vertical_collision ? "是 (Collision)" : "无";
            document.getElementById('debug-vertical-col').style.color = data.has_vertical_collision ? "#f44336" : "#4CAF50";
            
            document.getElementById('debug-below-preblocked').innerText = data.has_below_preblocked_failure ? "是 (Blocked Below)" : "无";
            document.getElementById('debug-below-preblocked').style.color = data.has_below_preblocked_failure ? "#f44336" : "#4CAF50";
            
            // 代价
            document.getElementById('debug-preblocked-cost').innerText = data.preblocked_cost.toFixed(3);
            document.getElementById('debug-risk-cost').innerText = data.risk_cost.toFixed(3);

            // 是否为候选点及可通行状态
            document.getElementById('debug-candidate').innerText = data.is_candidate ? "是 (Yes)" : "否 (No)";
            document.getElementById('debug-candidate').style.color = data.is_candidate ? "#4CAF50" : "#f44336";

            document.getElementById('debug-traversable').innerText = data.is_traversable ? "是 (Yes)" : "否 (No)";
            document.getElementById('debug-traversable').style.color = data.is_traversable ? "#4CAF50" : "#f44336";

            const nodeSourceEl = document.getElementById('debug-node-source');
            if (nodeSourceEl) {
                nodeSourceEl.innerText = data.node_source_info || "无";
            }

            // 缓存最新诊断数据以备复制
            latestDebugData = {
                world_x: x,
                world_y: y,
                world_z: z,
                ...data
            };
        } else {
            statusEl.innerText = `查询失败: ${data.message}`;
            latestDebugData = null;
        }
    } catch (err) {
        statusEl.innerText = `调试服务不可用: ${err.message}`;
    }
}

window.addEventListener('pointerdown', (event) => {
    if (event.clientX < 330) return; // 避开 UI 面板
    
    const params = getBrushParams();
    console.log("【点击事件】pointerdown 触发。当前工具:", params.tool, "点击键位:", event.button, "是否按空格:", isSpacePressed);
    
    if (params.tool === 'view' || isSpacePressed || event.button !== 0) {
        if (event.button === 2 || (event.button === 0 && (params.tool === 'view' || isSpacePressed))) {
            syncPitchYawFromCamera();
            isFPDragging = true;
            prevMouseX = event.clientX;
            prevMouseY = event.clientY;
            controls.enabled = false;
        }
        return;
    }
    
    // 如果是设置起点 / 终点工具，则执行单独的位置获取并发送给 ROS，而不触发持续画图
    if (params.tool === 'start' || params.tool === 'goal') {
        const hitTarget = getInteractionTarget();
        if (hitTarget) {
            let px, py, pz;
            if (hitTarget.type === 'voxel') {
                px = hitTarget.voxel.x;
                py = hitTarget.voxel.y;
                pz = hitTarget.voxel.z + hitTarget.scale[2]/2 + 0.3;
            } else {
                px = hitTarget.position.x;
                py = hitTarget.position.y;
                pz = hitTarget.position.z + 0.3;
            }
            if (params.tool === 'start') {
                setStartPoint(px, py, pz);
            } else {
                setGoalPoint(px, py, pz);
            }
        }
        return;
    }
    
    if (params.tool === 'debug' || params.tool === 'debug_air') {
        const hitTarget = getInteractionTarget();
        if (hitTarget) {
            let px, py, pz;
            if (hitTarget.type === 'voxel') {
                if (params.tool === 'debug') {
                    // 调试已有体素：精确查询被点击体素本身的坐标
                    px = hitTarget.voxel.x;
                    py = hitTarget.voxel.y;
                    pz = hitTarget.voxel.z;
                } else {
                    // 调试空网格：查询表面相邻的空闲网格
                    px = hitTarget.voxel.x + hitTarget.normal.x * hitTarget.scale[0];
                    py = hitTarget.voxel.y + hitTarget.normal.y * hitTarget.scale[1];
                    pz = hitTarget.voxel.z + hitTarget.normal.z * hitTarget.scale[2];
                }
            } else {
                px = hitTarget.position.x;
                py = hitTarget.position.y;
                pz = hitTarget.position.z;
            }
            const layerName = hitTarget.layerName || '';
            queryCellDebugInfo(px, py, pz, layerName);
        }
        return;
    }
    
    isPainting = true;
    controls.enabled = false; // 绘图时禁用相机旋转
    
    activeHit = getInteractionTarget();
    console.log("【点击事件】获取目标:", activeHit);
    executePaint();
});

window.addEventListener('pointerup', () => {
    console.log("【点击事件】pointerup 释放绘图状态");
    isPainting = false;
    if (isFPDragging) {
        isFPDragging = false;
    }
    controls.enabled = true;
});

// ---- 4. 网络通信 (API 调用) ----

// 初始化记录地图版本的辅助函数
async function initMapVersions() {
    try {
        const res = await fetch('/api/map_version');
        if (res.ok) {
            const data = await res.json();
            serverPreblockedVersion = data.preblocked_version;
            serverOccupiedVersion = data.occupied_version;
        }
    } catch (e) {}
}

async function reloadMapFromServer(silent = false) {
    if (!silent) {
        statusEl.innerText = "正在加载...";
        showLoading("⏳ 正在载入地图数据并等待 C++ 预构建...");
    }
    
    // 如果是静默被动拉取，我们只查询当前缓存的地图（不触发底层磁盘重新加载服务）
    // 如果是主动加载，我们调用 load_map 触发底层地图包的完整载入服务
    const activeLayers = getActiveRequestedLayers();
    const queryStr = (silent && activeLayers.length > 0) ? `?layers=${activeLayers.join(',')}` : '';
    const url = silent ? `/api/get_current_map${queryStr}` : '/api/load_map';
    const method = silent ? 'GET' : 'POST';
    
    const req = {
        root_path: document.getElementById('root-path').value,
        map_name: document.getElementById('map-name').value
    };
    
    try {
        const fetchOptions = {
            method: method,
            headers: { 'Content-Type': 'application/json' }
        };
        if (method === 'POST') {
            fetchOptions.body = JSON.stringify(req);
        }
        
        const res = await fetch(url, fetchOptions);
        if (!res.ok) throw new Error((await res.json()).detail || "获取地图数据失败");
        const data = await res.json();
        
        // 恢复数据（传入对应的地图分辨率尺度与多尺寸节点组）
        if (data.layers.occupied) layers.occupied.loadFromArray(data.layers.occupied.points, data.layers.occupied.scale, null, data.layers.occupied.groups);
        if (data.layers.preblocked) layers.preblocked.loadFromArray(data.layers.preblocked.points, data.layers.preblocked.scale, null, data.layers.preblocked.groups);
        if (data.layers.traversable) layers.traversable.loadFromArray(data.layers.traversable.points, data.layers.traversable.scale, null, data.layers.traversable.groups);
        if (data.layers.risk_cost) layers.risk_cost.loadFromArray(data.layers.risk_cost.points, data.layers.risk_cost.scale, data.layers.risk_cost.intensities, data.layers.risk_cost.groups);
        if (data.layers.local_octomap) layers.local_octomap.loadFromArray(data.layers.local_octomap.points, data.layers.local_octomap.scale, null, data.layers.local_octomap.groups);
        if (data.layers.fused_octomap) layers.fused_octomap.loadFromArray(data.layers.fused_octomap.points, data.layers.fused_octomap.scale, null, data.layers.fused_octomap.groups);
        if (data.layers.emergency_stop_free) layers.emergency_stop_free.loadFromArray(data.layers.emergency_stop_free.points, data.layers.emergency_stop_free.scale, null, data.layers.emergency_stop_free.groups);
        if (data.layers.emergency_stop_occupied) layers.emergency_stop_occupied.loadFromArray(data.layers.emergency_stop_occupied.points, data.layers.emergency_stop_occupied.scale, null, data.layers.emergency_stop_occupied.groups);
        
        const sizeOcc = data.layers.occupied ? data.layers.occupied.points.length : 0;
        const sizePre = data.layers.preblocked ? data.layers.preblocked.points.length : 0;
        const sizeTrav = data.layers.traversable ? data.layers.traversable.points.length : 0;
        const sizeRisk = data.layers.risk_cost ? data.layers.risk_cost.points.length : 0;
        console.log("【地图重载成功】数据量：占据(橙):", sizeOcc, "禁行(红):", sizePre, "可通行(绿):", sizeTrav, "通行代价(渐变):", sizeRisk);
        
        // 自动将视角与绘制高度对齐到点云几何中心
        if (data.layers.occupied && data.layers.occupied.points.length > 0) {
            const pts = data.layers.occupied.points;
            let sumX = 0, sumY = 0, sumZ = 0;
            for (let i = 0; i < pts.length; i++) {
                sumX += pts[i][0];
                sumY += pts[i][1];
                sumZ += pts[i][2];
            }
            const avgX = sumX / pts.length;
            const avgY = sumY / pts.length;
            const avgZ = sumZ / pts.length;

            // 1. 将旋转中心（Orbit Target）设定为地图中心
            controls.target.set(avgX, avgY, avgZ);

            // 2. 如果是主动加载地图（非静默背景同步），将相机位置平移对齐到地图上方
            if (!silent) {
                camera.position.set(avgX, avgY - 25, avgZ + 25);
                
                // 3. 自动将初始绘图高度对齐到点云的平均高度，更新 UI 和参考平面
                const resolutionZ = data.layers.occupied.scale[2] || 0.05;
                zHeight = Math.round(avgZ / resolutionZ) * resolutionZ;
                document.getElementById('edit-z').value = zHeight.toFixed(2);
                editPlane.position.z = zHeight;
            }
            controls.update();
        }
        
        if (!silent) {
            statusEl.innerText = `加载成功! 占据: ${sizeOcc}, 禁行: ${sizePre}, 可通行: ${sizeTrav}, 代价点数: ${sizeRisk}`;
        }
        
        // 同步当前的服务器地图版本号，防止拉取完立即又检测出变化
        await initMapVersions();
        
        // 缓存成功的地图路径和名字
        localStorage.setItem('map_root_path', req.root_path);
        localStorage.setItem('map_name', req.map_name);
        
        isDirty = false; // 重置脏标记
        isPreblockedDirty = false;
        return true;
    } catch (err) {
        if (!silent) statusEl.innerText = `加载失败: ${err.message}`;
        if (!silent) hideLoading();
        return false;
    } finally {
        if (!silent) {
            requestAnimationFrame(() => {
                requestAnimationFrame(() => {
                    hideLoading();
                });
            });
        }
    }
}

document.getElementById('btn-load').addEventListener('click', () => reloadMapFromServer(false));

async function sendMapData(endpoint) {
    const btnSync = document.getElementById('btn-sync');
    const btnSave = document.getElementById('btn-save');
    const btnLoad = document.getElementById('btn-load');
    
    btnSync.disabled = true;
    btnSave.disabled = true;
    btnLoad.disabled = true;
    const isSync = endpoint.includes('sync_ros');
    const loadingText = isSync ? "⏳ 正在同步地图至 ROS 并等待 C++ 结算，请稍候..." : "⏳ 正在保存地图文件，请稍候...";
    statusEl.innerText = isSync ? "正在同步地图至 ROS 并等待 C++ 结算，请稍候..." : "正在保存地图，请稍候...";
    showLoading(loadingText);
    
    const layersData = {
        occupied: { points: layers.occupied.getArray(), scale: layers.occupied.scale }
    };
    if (isPreblockedDirty) {
        layersData.preblocked = { points: layers.preblocked.getArray(), scale: layers.preblocked.scale };
    }
    const req = {
        root_path: document.getElementById('root-path').value,
        map_name: document.getElementById('map-name').value,
        layers: layersData
    };
    try {
        const res = await fetch(endpoint, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(req)
        });
        if (!res.ok) throw new Error(await res.text());
        const data = await res.json();
        statusEl.innerText = data.message;
        
        // 同步成功后，我们将本地脏标记清除
        isDirty = false;
        isPreblockedDirty = false;
        
        // 如果是同步到 ROS，同步成功后立即主动拉取一次 C++ 重新计算后的最新图层
        if (endpoint.includes('sync_ros')) {
            await reloadMapFromServer(true);
        }
    } catch (err) {
        statusEl.innerText = `请求失败: ${err.message}`;
    } finally {
        btnSync.disabled = false;
        btnSave.disabled = false;
        btnLoad.disabled = false;
        hideLoading();
    }
}

// 同步按钮
document.getElementById('btn-sync').addEventListener('click', () => sendMapData('/api/sync_ros'));
// 保存按钮
document.getElementById('btn-save').addEventListener('click', () => sendMapData('/api/save_map'));

// 复制诊断信息按钮
if (btnCopyDebug) {
    btnCopyDebug.addEventListener('click', async () => {
        if (!latestDebugData) {
            statusEl.innerText = "请先在地图上点击网格进行调试查询！";
            return;
        }
        
        const now = new Date();
        const timeStr = `${now.getFullYear()}/${String(now.getMonth()+1).padStart(2,'0')}/${String(now.getDate()).padStart(2,'0')} ${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}`;
        
        const reasonMap = {
            "none": "无",
            "manual": "手动绘制禁行",
            "step_or_obstacle_edge": "台阶或障碍物边缘",
            "cliff_or_suspended": "悬空或悬崖边缘"
        };
        const cnReason = reasonMap[latestDebugData.preblocked_reason] || latestDebugData.preblocked_reason;

        let occupancyStr = "否 (Free)";
        if (latestDebugData.is_unknown) {
            occupancyStr = "未知 (Unknown)";
        } else if (latestDebugData.is_occupied) {
            occupancyStr = "是 (Occupied)";
        }

        const report = `### 网格调试诊断报告 ###
- 查询时间: ${timeStr}
- 网格坐标: [${latestDebugData.grid_x}, ${latestDebugData.grid_y}, ${latestDebugData.grid_z}]
- 世界坐标: [${latestDebugData.world_x.toFixed(2)}, ${latestDebugData.world_y.toFixed(2)}, ${latestDebugData.world_z.toFixed(2)}]
- 是否占据 (Occupied): ${occupancyStr}
- 地面支撑 (Ground Support): ${latestDebugData.has_ground_support ? "是 (Supported)" : "否 (No Support)"}
- 是否禁行 (Preblocked): ${latestDebugData.is_preblocked ? "是" : "否"} (原因: ${cnReason})
- 水平碰撞 (Horizontal Collision): ${latestDebugData.has_horizontal_collision ? "是 (Collision)" : "无"}
- 垂直碰撞 (Vertical Collision): ${latestDebugData.has_vertical_collision ? "是 (Collision)" : "无"}
- 下方禁行阻断 (Below Preblocked): ${latestDebugData.has_below_preblocked_failure ? "是 (Blocked Below)" : "无"}
- 禁行代价 (Preblocked Cost): ${latestDebugData.preblocked_cost.toFixed(3)}
- 风险代价 (Risk Cost): ${latestDebugData.risk_cost.toFixed(3)}
- 是否为候选点 (Is Candidate): ${latestDebugData.is_candidate ? "是" : "否"}
- 可通行列表中 (Is Traversable): ${latestDebugData.is_traversable ? "是" : "否"}
- 八叉树节点诊断 (Node Source): ${latestDebugData.node_source_info || "无"}`;

        try {
            await navigator.clipboard.writeText(report);
            const originalText = btnCopyDebug.innerText;
            btnCopyDebug.innerText = "已复制！";
            btnCopyDebug.style.background = "#4CAF50";
            statusEl.innerText = "诊断报告已成功复制到剪贴板！";
            setTimeout(() => {
                btnCopyDebug.innerText = originalText;
                btnCopyDebug.style.background = "#2196F3";
            }, 1500);
        } catch (err) {
            statusEl.innerText = `复制失败: ${err.message}`;
        }
    });
}

// 阻止点击调试面板时事件向上传播，防止点击穿透到场景中
if (debugPanelDiv) {
    ['pointerdown', 'pointerup', 'mousedown', 'mouseup', 'click'].forEach(evtName => {
        debugPanelDiv.addEventListener(evtName, (e) => {
            e.stopPropagation();
        });
    });
}

// 键盘移动步长 (米)
const moveSpeed = 0.2;

function handleKeyboardMovement() {
    if (isInputFocused()) return;
    
    // 获取相机的朝向向量
    const forward = new THREE.Vector3();
    camera.getWorldDirection(forward);
    
    // 将朝向向量投影到水平面 (XY 面)，以进行直观的水平面平移
    forward.z = 0;
    forward.normalize();
    
    // 计算相机的右方向量
    const right = new THREE.Vector3();
    right.crossVectors(forward, camera.up).normalize();

    const translation = new THREE.Vector3(0, 0, 0);

    // W/S: 前进/后退
    if (keysPressed['KeyW'] || keysPressed['ArrowUp']) {
        translation.addScaledVector(forward, moveSpeed);
    }
    if (keysPressed['KeyS'] || keysPressed['ArrowDown']) {
        translation.addScaledVector(forward, -moveSpeed);
    }
    // A/D: 左移/右移
    if (keysPressed['KeyA'] || keysPressed['ArrowLeft']) {
        translation.addScaledVector(right, -moveSpeed);
    }
    if (keysPressed['KeyD'] || keysPressed['ArrowRight']) {
        translation.addScaledVector(right, moveSpeed);
    }
    // E/Q: 上升/下降 (沿着世界坐标 Z 轴)
    if (keysPressed['KeyE']) {
        translation.z += moveSpeed;
    }
    if (keysPressed['KeyQ']) {
        translation.z -= moveSpeed;
    }

    // 同时平移相机位置和控制器焦点，防止缩放锁死
    if (translation.lengthSq() > 0) {
        camera.position.add(translation);
        controls.target.add(translation);
    }
}

// 动画循环
function animate() {
    requestAnimationFrame(animate);
    handleKeyboardMovement();
    controls.update();
    renderer.render(scene, camera);
}
animate();