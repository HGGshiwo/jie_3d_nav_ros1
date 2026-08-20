import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---- 0. 编辑状态与地图版本控制 ----
let isDirty = false;
let isPreblockedDirty = false;
let serverPreblockedVersion = 0;
let serverOccupiedVersion = 0;

// ---- 1. 场景初始化 ----
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

// Z轴向上的相机设置
const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 2000);
camera.position.set(0, -25, 25);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
// 视角控制键配置：
// - 左键：旋转（仅在非绘图状态或视角工具下生效）
// - 中键：平移（任何状态下均可使用）
// - 右键：旋转（任何状态下均可使用，方便绘图时随时微调视角）
controls.mouseButtons = {
    LEFT: THREE.MOUSE.ROTATE,
    MIDDLE: THREE.MOUSE.PAN,
    RIGHT: THREE.MOUSE.ROTATE
};
controls.minDistance = 1.0; // 限制最小缩放距离，防止穿过中心点导致消失
controls.maxDistance = 500.0; // 限制最大缩放距离

// 监听复选框，切换图层显隐
document.getElementById('show-occupied').addEventListener('change', (e) => {
    layers.occupied.mesh.visible = e.target.checked;
});
document.getElementById('show-preblocked').addEventListener('change', (e) => {
    layers.preblocked.mesh.visible = e.target.checked;
});

scene.add(new THREE.AmbientLight(0xffffff, 0.8));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
dirLight.position.set(10, 10, 20);
scene.add(dirLight);

// 辅助网格
const gridHelper = new THREE.GridHelper(50, 50, 0x444444, 0x222222);
gridHelper.rotation.x = Math.PI / 2;
scene.add(gridHelper);

// 编辑参考平面 (不可见，用于接收鼠标射线检测)
const planeGeo = new THREE.PlaneGeometry(1000, 1000);
const planeMat = new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide });
const editPlane = new THREE.Mesh(planeGeo, planeMat);
scene.add(editPlane);

// ---- 2. 高性能体素管理系统 ----
const MAX_INSTANCES = 500000; 
const LAYER_COLORS = {
    "occupied": 0xf27327, // 橙色
    "preblocked": 0xff0000, // 红色
    "traversable": 0x00ff00  // 绿色 (仅显示，通常不手绘)
};

class LayerManager {
    constructor(layerName, scale) {
        this.layerName = layerName;
        this.scale = scale; // [x, y, z]
        this.voxelMap = new Map(); // key: "x,y,z", value: {x,y,z}
        this.voxelList = []; // 新建缓存扁平数组，供 O(1) 快速查询击中体素用
        
        // 创建 1x1x1 的单位几何体，并在渲染实例时动态应用 scale
        const geometry = new THREE.BoxGeometry(0.95, 0.95, 0.95);
        
        // 关键修复：给 base geometry 赋予极大的包围盒和包围球，
        // 绕过 Three.js 默认仅以原点几何体尺寸 (0.82m) 进行射线交点早期过滤的机制。
        // 这能确保偏离原点位置的体素实例也能够正常被射线射中并高亮/擦除。
        geometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 99999);
        geometry.boundingBox = new THREE.Box3(new THREE.Vector3(-99999, -99999, -99999), new THREE.Vector3(99999, 99999, 99999));
        
        const material = new THREE.MeshLambertMaterial({ color: LAYER_COLORS[layerName] || 0xffffff });
        
        // 使用 InstancedMesh 渲染，性能极致
        this.mesh = new THREE.InstancedMesh(geometry, material, MAX_INSTANCES);
        this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        this.mesh.count = 0;
        
        // 关键修复：设置 InstancedMesh 本身的包围球与包围盒。
        // Three.js 在对 InstancedMesh 求交时会缓存并使用此处的包围球，
        // 如果最初为 null，它会计算出空的包围球并缓存。后续添加体素后该缓存不会自动刷新，
        // 从而导致所有后续射线求交由于 intersectsSphere 失败而直接返回 0。
        this.mesh.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 99999);
        this.mesh.boundingBox = new THREE.Box3(new THREE.Vector3(-99999, -99999, -99999), new THREE.Vector3(99999, 99999, 99999));
        
        scene.add(this.mesh);
        
        this.dummy = new THREE.Object3D();
    }

    hash(x, y, z) {
        // 防止浮点数精度问题
        return `${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)}`;
    }

    addVoxel(x, y, z) {
        const key = this.hash(x, y, z);
        if (!this.voxelMap.has(key) && this.voxelMap.size < MAX_INSTANCES) {
            this.voxelMap.set(key, {x, y, z});
            this.updateInstancedMesh();
            isDirty = true; // 产生了本地编辑修改，标记为脏状态
            if (this.layerName === 'preblocked') {
                isPreblockedDirty = true;
            }
            return true;
        }
        return false;
    }

    removeVoxel(x, y, z) {
        const key = this.hash(x, y, z);
        if (this.voxelMap.has(key)) {
            this.voxelMap.delete(key);
            this.updateInstancedMesh();
            isDirty = true; // 产生了本地编辑修改，标记为脏状态
            if (this.layerName === 'preblocked') {
                isPreblockedDirty = true;
            }
            return true;
        }
        return false;
    }

    updateInstancedMesh() {
        let i = 0;
        this.voxelList = []; // 每次更新时同步构建扁平数组
        this.voxelMap.forEach((pos) => {
            this.voxelList.push(pos);
            if (i < MAX_INSTANCES) {
                this.dummy.position.set(pos.x, pos.y, pos.z);
                this.dummy.scale.set(this.scale[0], this.scale[1], this.scale[2]);
                this.dummy.updateMatrix();
                this.mesh.setMatrixAt(i++, this.dummy.matrix);
            }
        });
        this.mesh.count = Math.min(this.voxelMap.size, MAX_INSTANCES);
        this.mesh.instanceMatrix.needsUpdate = true;
    }

    loadFromArray(points, scale) {
        if (scale) {
            this.scale = scale;
        }
        this.voxelMap.clear();
        points.forEach(pt => {
            this.voxelMap.set(this.hash(pt[0], pt[1], pt[2]), {x: pt[0], y: pt[1], z: pt[2]});
        });
        this.updateInstancedMesh();
    }
    
    getArray() {
        return this.voxelList.map(p => [p.x, p.y, p.z]);
    }
    
    getVoxelByInstanceId(instanceId) {
        return this.voxelList[instanceId];
    }
}

// 容器 (默认分辨率初始化为 0.4)
let layers = {
    occupied: new LayerManager("occupied", [0.4, 0.4, 0.4]),
    preblocked: new LayerManager("preblocked", [0.4, 0.4, 0.4])
};

// ---- 3. 编辑交互逻辑 ----
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
let isPainting = false;

// 游标提示框（单位尺寸几何体，动态应用 scale）
const cursorGeo = new THREE.BoxGeometry(1.0, 1.0, 1.0);
const cursorMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
const cursor = new THREE.Mesh(cursorGeo, cursorMat);
scene.add(cursor);

function getBrushParams() {
    return {
        tool: document.querySelector('input[name="tool"]:checked').value,
        layer: document.getElementById('edit-layer').value,
        size: parseInt(document.getElementById('brush-size').value),
        zHeight: parseFloat(document.getElementById('edit-z').value)
    };
}

// 同步检测平面的高度
document.getElementById('edit-z').addEventListener('change', (e) => {
    editPlane.position.z = parseFloat(e.target.value);
});

let activeHit = null; // 存储当前鼠标射线击中的目标

function getInteractionTarget() {
    const params = getBrushParams();
    const activeLayerName = params.layer;
    const targetLayer = layers[activeLayerName];
    const scale = targetLayer.scale;

    raycaster.setFromCamera(mouse, camera);

    // 1. 尝试与场景中所有显示的体素求交
    const meshesToIntersect = [];
    if (layers.occupied.mesh.visible) meshesToIntersect.push(layers.occupied.mesh);
    if (layers.preblocked.mesh.visible) meshesToIntersect.push(layers.preblocked.mesh);

    const voxelIntersects = raycaster.intersectObjects(meshesToIntersect);
    
    if (params.tool === 'eraser') {
        console.log("【橡皮擦调试】可求交网格数:", meshesToIntersect.length, "；射线交点个数:", voxelIntersects.length);
    }
    
    if (voxelIntersects.length > 0) {
        const hit = voxelIntersects[0];
        const hitMesh = hit.object;
        const instanceId = hit.instanceId;
        
        let hitLayerName = null;
        if (hitMesh === layers.occupied.mesh) hitLayerName = 'occupied';
        else if (hitMesh === layers.preblocked.mesh) hitLayerName = 'preblocked';
        
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
                // 物体无旋转，故世界法向等于局部法向
                return {
                    type: 'voxel',
                    layerName: hitLayerName,
                    voxel: voxel,
                    normal: normal,
                    scale: layers[hitLayerName].scale
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
            const hitLayer = layers[activeHit.layerName];
            
            for (let dx = -half; dx <= half; dx++) {
                for (let dy = -half; dy <= half; dy++) {
                    const px = cx + dx * activeHit.scale[0];
                    const py = cy + dy * activeHit.scale[1];
                    // 同时擦除占据层（橙色）和禁行层（红色），避免单独残留导致C++端根据另一层重新计算恢复
                    layers.occupied.removeVoxel(px, py, cz);
                    layers.preblocked.removeVoxel(px, py, cz);
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
                targetLayer.addVoxel(px, py, cz);
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

// ---- 3.1 键盘空格键快捷切视角 ----
let isSpacePressed = false;
window.addEventListener('keydown', (e) => {
    if (e.code === 'Space') {
        isSpacePressed = true;
        document.body.style.cursor = 'grab';
        cursor.visible = false;
    }
});
window.addEventListener('keyup', (e) => {
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

// 轮询获取规划路径
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
        // 忽略网络错误，避免刷屏
    }
}, 1000);

// 页面加载时自动获取后端默认的地图配置并加载，避免手动重新输入
async function initDefaultMap() {
    try {
        const res = await fetch('/api/default_map');
        if (res.ok) {
            const data = await res.json();
            if (data.root_path && data.map_name) {
                document.getElementById('root-path').value = data.root_path;
                document.getElementById('map-name').value = data.map_name;
                await reloadMapFromServer(false);
            }
        }
    } catch (err) {
        console.error("Failed to load default map config:", err);
    }
}
initDefaultMap();

let lastMoveTime = 0;
window.addEventListener('pointermove', (event) => {
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
    
    activeHit = getInteractionTarget();
    updateCursorVisual();

    if (isPainting) {
        executePaint();
    }
});

window.addEventListener('pointerdown', (event) => {
    if (event.clientX < 330) return; // 避开 UI 面板
    
    const params = getBrushParams();
    console.log("【点击事件】pointerdown 触发。当前工具:", params.tool, "点击键位:", event.button, "是否按空格:", isSpacePressed);
    
    if (params.tool === 'view' || isSpacePressed || event.button !== 0) {
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
    
    isPainting = true;
    controls.enabled = false; // 绘图时禁用相机旋转
    
    activeHit = getInteractionTarget();
    console.log("【点击事件】获取目标:", activeHit);
    executePaint();
});

window.addEventListener('pointerup', () => {
    console.log("【点击事件】pointerup 释放绘图状态");
    isPainting = false;
    controls.enabled = true;
});

// ---- 4. 网络通信 (API 调用) ----
const statusEl = document.getElementById('status');

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
    if (!silent) statusEl.innerText = "正在加载...";
    
    // 如果是静默被动拉取，我们只查询当前缓存的地图（不触发底层磁盘重新加载服务）
    // 如果是主动加载，我们调用 load_map 触发底层地图包的完整载入服务
    const url = silent ? '/api/get_current_map' : '/api/load_map';
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
        
        // 恢复数据（传入对应的地图分辨率尺度）
        if (data.layers.occupied) layers.occupied.loadFromArray(data.layers.occupied.points, data.layers.occupied.scale);
        if (data.layers.preblocked) layers.preblocked.loadFromArray(data.layers.preblocked.points, data.layers.preblocked.scale);
        
        const sizeOcc = data.layers.occupied ? data.layers.occupied.points.length : 0;
        const sizePre = data.layers.preblocked ? data.layers.preblocked.points.length : 0;
        console.log("【地图重载成功】数据量：占据(橙):", sizeOcc, "禁行(红):", sizePre);
        
        if (!silent) {
            statusEl.innerText = `加载成功! 占据: ${sizeOcc}, 禁行: ${sizePre}`;
        }
        
        // 同步当前的服务器地图版本号，防止拉取完立即又检测出变化
        await initMapVersions();
        isDirty = false; // 重置脏标记
        isPreblockedDirty = false;
        return true;
    } catch (err) {
        if (!silent) statusEl.innerText = `加载失败: ${err.message}`;
        return false;
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
    statusEl.innerText = endpoint.includes('sync_ros') ? "正在同步地图至 ROS 并等待 C++ 结算，请稍候..." : "正在保存地图，请稍候...";
    
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
    }
}

// 同步按钮
document.getElementById('btn-sync').addEventListener('click', () => sendMapData('/api/sync_ros'));
// 保存按钮
document.getElementById('btn-save').addEventListener('click', () => sendMapData('/api/save_map'));

// 视角调整功能
function setView(viewType) {
    controls.reset();
    if (viewType === 'default') {
        camera.position.set(0, -25, 25);
        camera.up.set(0, 0, 1);
        controls.target.set(0, 0, 0);
    } else if (viewType === 'top') {
        camera.position.set(0, -0.01, 35);
        camera.up.set(0, 0, 1); // 统一保持 Z 轴向上，使用极微小的 Y 轴偏移防止万向锁
        controls.target.set(0, 0, 0);
    } else if (viewType === 'front') {
        camera.position.set(0, -35, 0);
        camera.up.set(0, 0, 1);
        controls.target.set(0, 0, 0);
    } else if (viewType === 'side') {
        camera.position.set(35, 0, 0);
        camera.up.set(0, 0, 1);
        controls.target.set(0, 0, 0);
    }
    controls.update();
}

document.getElementById('btn-view-default').addEventListener('click', () => setView('default'));
document.getElementById('btn-view-top').addEventListener('click', () => setView('top'));
document.getElementById('btn-view-front').addEventListener('click', () => setView('front'));
document.getElementById('btn-view-side').addEventListener('click', () => setView('side'));

// 窗口适配
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// 动画循环
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();