import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/**
 * Initializes the Three.js 3D scene environment.
 * @param {HTMLElement} container The container to append the WebGL renderer.
 * @returns {Object} { scene, camera, renderer, controls, editPlane }
 */
export function initScene(container) {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a1a);

    // Camera setup with Z-axis upwards
    const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 2000);
    camera.position.set(0, -25, 25);
    camera.up.set(0, 0, 1);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.mouseButtons = {
        LEFT: THREE.MOUSE.ROTATE,
        MIDDLE: THREE.MOUSE.PAN,
        RIGHT: THREE.MOUSE.ROTATE
    };
    controls.minDistance = 1.0;
    controls.maxDistance = 500.0;

    // --- Enhanced Lights for Better Spatial Sense ---
    // Lower ambient light to prevent flatness
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.35);
    scene.add(ambientLight);

    // Strong primary directional light for clear shading contrast
    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.75);
    dirLight1.position.set(15, 10, 30);
    scene.add(dirLight1);

    // Soft secondary light for filling dark back-faces
    const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.25);
    dirLight2.position.set(-15, -10, -10);
    scene.add(dirLight2);

    // Ground Helper Grid
    const gridHelper = new THREE.GridHelper(50, 50, 0x444444, 0x222222);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    // Invisible reference plane for mouse drawing raycast
    const planeGeo = new THREE.PlaneGeometry(1000, 1000);
    const planeMat = new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide });
    const editPlane = new THREE.Mesh(planeGeo, planeMat);
    scene.add(editPlane);

    // Auto resize handling
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    return { scene, camera, renderer, controls, editPlane };
}
