import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { MTLLoader } from "three/examples/jsm/loaders/MTLLoader.js";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

// --- Asset Bundle Manifest Imports ---
import p1AssetUrl from "../assets/P1.obj?url";
import p1MaterialUrl from "../assets/P1.mtl?url";
import p2AssetUrl from "../assets/P2.obj?url";
import p2MaterialUrl from "../assets/P2.mtl?url";
import p3AssetUrl from "../assets/P3.obj?url";
import p3MaterialUrl from "../assets/P3.mtl?url";
import p4AssetUrl from "../assets/P4.obj?url";
import p4MaterialUrl from "../assets/P4.mtl?url";
import p5AssetUrl from "../assets/P5.obj?url";
import p5MaterialUrl from "../assets/P5.mtl?url";
import p6AssetUrl from "../assets/P6.obj?url";
import p6MaterialUrl from "../assets/P6.mtl?url";

export interface ArmState {
  base: number;
  shoulder: number;
  elbow: number;
  wrist: number;
  camera: number;
  temperature: number;
}

// =========================================================================
//  📍 PRE-SET POSE CONFIGURATION LINES (Updated Home Wrist to 90)
// =========================================================================
export const ARM_PRESETS = {
  home: { base: 0, shoulder: 0, elbow: 0, wrist: 90, camera: 0, temperature: 25 },
  pickup: { base: 90, shoulder: -45, elbow: 45, wrist: 90, camera: 0, temperature: 25 },
  rest: { base: 180, shoulder: 30, elbow: -30, wrist: 0, camera: 0, temperature: 25 },
};

interface Props {
  state: ArmState;
}

type Vec3 = [number, number, number];

const RAD = Math.PI / 180;
const SPEED = 0.12;
const HINGE_DOWN = 0.8;

// --- Rigging Kinematics Framework ---
const ARM_PARTS = {
  p1: { position: [0.0, 0.0, 0.0] as Vec3, rotation: [-1.571, 0.0, 0.0] as Vec3 },
  p2: {
    position: [0.3, 7.0, -2.5] as Vec3,
    rotation: [-1.571, 0.0, 0.0] as Vec3,
    hinge: [-0.3, 7.0 - HINGE_DOWN, -1.801] as Vec3,
    swingAxis: [0, 1, 0] as Vec3,
    angleSign: 1,
    connectsTo: "p3",
  },
  p3: {
    position: [-0.231, 8.5, -1.801] as Vec3,
    rotation: [-1.571, 0.0, 0.0] as Vec3,
    hinge: [-0.231, 8.519 - HINGE_DOWN, -1.801] as Vec3,
    swingAxis: [1, 0, 0] as Vec3,
    angleSign: -1,
    connectsTo: "p4",
  },
  p4: {
    position: [0.274, 19.184, -0.281] as Vec3,
    rotation: [-1.571, 0.0, 0.0] as Vec3,
    hinge: [0.274, 19.00 - HINGE_DOWN, -1.75] as Vec3, 
    swingAxis: [1, 0, 0] as Vec3,
    angleSign: 1,
    connectsTo: "p5",
  },
  p5: {
    position: [-0.4, 32.79, -1.6] as Vec3,
    rotation: [-1.571, 0.0, 0.0] as Vec3,
    hinge: [0.85, 33.32 - HINGE_DOWN, -1.6] as Vec3,
    swingAxis: [1, 0, 0] as Vec3,
    angleSign: 1,
    connectsTo: "p6", 
  },
  p6: {
    position: [-4.170, 40, -8.910] as Vec3,
    hinge: [-1.440, 41.5, -1.630] as Vec3,
    rotation: [1.571, 0.0, 0.0] as Vec3,
    swingAxis: [0, 1, 0] as Vec3,
    angleSign: 1,
    connectsTo: "none", 
  },
};

function toVector(value: Vec3) {
  return new THREE.Vector3(...value);
}

function offset(from: Vec3, to: Vec3) {
  return toVector(to).sub(toVector(from));
}

export function ArmScene({ state }: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const stateRef = useRef(state);
  stateRef.current = state;

  const currentAnglesRef = useRef({
    base: state.base,
    shoulder: state.shoulder,
    elbow: state.elbow,
    wrist: state.wrist,
    p6: state.camera, 
    camera: state.camera,
  });

  useEffect(() => {
    const container = containerRef.current!;
    
    const width = container.clientWidth || 800;
    const height = container.clientHeight || 600;

    const scene = new THREE.Scene();
    scene.background = null;

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 2000);
    camera.position.set(60, 45, 70);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0, 20, 0);

    scene.add(new THREE.AmbientLight(0xffffff, 0.6));

    const sunPrimary = new THREE.DirectionalLight(0xffffff, 1.5);
    sunPrimary.position.set(80, 180, 100);
    sunPrimary.castShadow = true;
    scene.add(sunPrimary);

    const sunRim = new THREE.DirectionalLight(0xffe0b3, 0.6);
    sunRim.position.set(-80, 120, -100);
    scene.add(sunRim);

    const grid = new THREE.GridHelper(150, 50, 0xffb84d, 0x34384a);
    (grid.material as THREE.Material).transparent = true;
    (grid.material as THREE.Material).opacity = 0.35;
    scene.add(grid);

    const liveMaterials: THREE.MeshStandardMaterial[] = [];

    const processMtlToPbr = (mesh: THREE.Mesh) => {
      if (!mesh.material) return;
      let sourceColor = new THREE.Color(0x7f7f7f);
      
      if (Array.isArray(mesh.material)) {
        const mat = mesh.material[0] as any;
        if (mat && mat.color) sourceColor.copy(mat.color);
      } else {
        const mat = mesh.material as any;
        if (mat && mat.color) sourceColor.copy(mat.color);
      }

      const pbrMat = new THREE.MeshStandardMaterial({
        color: sourceColor,
        metalness: 0.4,
        roughness: 0.5,
        emissive: new THREE.Color(0x000000),
        emissiveIntensity: 0,
      });

      mesh.material = pbrMat;
      liveMaterials.push(pbrMat);
    };

    const loadFixedPartWithMtl = (objUrl: string, mtlUrl: string, parent: THREE.Object3D, position: Vec3, rotation: Vec3) => {
      const mtlLoader = new MTLLoader();
      mtlLoader.load(mtlUrl, (materials) => {
        materials.preload();
        const objLoader = new OBJLoader();
        objLoader.setMaterials(materials);
        objLoader.load(objUrl, (obj) => {
          obj.traverse((child) => {
            const mesh = child as THREE.Mesh;
            if (mesh.isMesh) {
              mesh.castShadow = true;
              mesh.receiveShadow = true;
              processMtlToPbr(mesh);
            }
          });
          obj.position.set(...position);
          obj.rotation.set(...rotation);
          parent.add(obj);
        });
      });
    };

    const loadMovingPartWithMtl = (
      objUrl: string,
      mtlUrl: string,
      joint: THREE.Group,
      part: { position: Vec3; rotation: Vec3; hinge: Vec3 },
      meshScale = 1.0
    ) => {
      const mtlLoader = new MTLLoader();
      mtlLoader.load(mtlUrl, (materials) => {
        materials.preload();
        const objLoader = new OBJLoader();
        objLoader.setMaterials(materials);
        objLoader.load(objUrl, (obj) => {
          obj.traverse((child) => {
            const mesh = child as THREE.Mesh;
            if (mesh.isMesh) {
              mesh.castShadow = true;
              mesh.receiveShadow = true;
              processMtlToPbr(mesh);
            }
          });

          if (meshScale !== 1.0) {
            obj.scale.setScalar(meshScale);
          }

          obj.rotation.set(...part.rotation);
          const calculatedOffset = offset(part.hinge, part.position);
          obj.position.copy(calculatedOffset.multiplyScalar(meshScale));

          joint.add(obj);
        });
      });
    };

    loadFixedPartWithMtl(p1AssetUrl, p1MaterialUrl, scene, ARM_PARTS.p1.position, ARM_PARTS.p1.rotation);

    const jBase = new THREE.Group();
    jBase.position.set(...ARM_PARTS.p2.hinge);
    scene.add(jBase);             

    const jShoulder = new THREE.Group();
    jShoulder.position.copy(offset(ARM_PARTS.p2.hinge, ARM_PARTS.p3.hinge));
    jBase.add(jShoulder);          

    const jElbow = new THREE.Group();
    jElbow.position.copy(offset(ARM_PARTS.p3.hinge, ARM_PARTS.p4.hinge));
    jShoulder.add(jElbow);        

    const jWrist = new THREE.Group();
    jWrist.position.copy(offset(ARM_PARTS.p4.hinge, ARM_PARTS.p5.hinge));
    jElbow.add(jWrist);           

    const jP6 = new THREE.Group();
    jP6.position.copy(offset(ARM_PARTS.p5.hinge, ARM_PARTS.p6.hinge));
    jWrist.add(jP6);

    const jCamera = new THREE.Group();
    jCamera.position.copy(offset(ARM_PARTS.p5.hinge, [0, 42 - HINGE_DOWN, -1.6]));
    jWrist.add(jCamera);          

    loadMovingPartWithMtl(p2AssetUrl, p2MaterialUrl, jBase, ARM_PARTS.p2);
    loadMovingPartWithMtl(p3AssetUrl, p3MaterialUrl, jShoulder, ARM_PARTS.p3);
    loadMovingPartWithMtl(p4AssetUrl, p4MaterialUrl, jElbow, ARM_PARTS.p4); 
    loadMovingPartWithMtl(p5AssetUrl, p5MaterialUrl, jWrist, ARM_PARTS.p5);
    loadMovingPartWithMtl(p6AssetUrl, p6MaterialUrl, jP6, ARM_PARTS.p6, 0.1); 

    const tBase = new THREE.Quaternion();
    const tShoulder = new THREE.Quaternion();
    const tElbow = new THREE.Quaternion();
    const tWrist = new THREE.Quaternion();
    const tP6 = new THREE.Quaternion(); 
    const tCamera = new THREE.Quaternion();

    const baseAxis = toVector(ARM_PARTS.p2.swingAxis);
    const shoulderAxis = toVector(ARM_PARTS.p3.swingAxis);
    const elbowAxis = toVector(ARM_PARTS.p4.swingAxis);
    const wristAxis = toVector(ARM_PARTS.p5.swingAxis);
    const p6Axis = toVector(ARM_PARTS.p6.swingAxis); 
    const cameraAxis = new THREE.Vector3(0, 0, 1);

    let frameId = 0;

    const animate = () => {
      frameId = requestAnimationFrame(animate);

      const s = stateRef.current;
      const cur = currentAnglesRef.current;

      cur.base += (s.base - cur.base) * SPEED;
      cur.shoulder += (s.shoulder - cur.shoulder) * SPEED;
      cur.elbow += (s.elbow - cur.elbow) * SPEED;
      cur.wrist += (s.wrist - cur.wrist) * SPEED;
      cur.p6 += (s.camera - cur.p6) * SPEED; 
      cur.camera += (s.camera - cur.camera) * SPEED;

      tBase.setFromAxisAngle(baseAxis, cur.base * RAD);
      tShoulder.setFromAxisAngle(shoulderAxis, cur.shoulder * RAD);
      tElbow.setFromAxisAngle(elbowAxis, cur.elbow * RAD);
      tWrist.setFromAxisAngle(wristAxis, cur.wrist * RAD);
      tP6.setFromAxisAngle(p6Axis, cur.p6 * RAD); 
      tCamera.setFromAxisAngle(cameraAxis, cur.camera * RAD);

      jBase.quaternion.copy(tBase);
      jShoulder.quaternion.copy(tShoulder);
      jElbow.quaternion.copy(tElbow);
      jWrist.quaternion.copy(tWrist);
      jP6.quaternion.copy(tP6); 
      jCamera.quaternion.copy(tCamera);

      const factor = Math.min(Math.max((s.temperature - 25) / 75, 0), 1);
      const glow = new THREE.Color(0x000000).lerp(new THREE.Color(0xff1100), factor);

      liveMaterials.forEach((m) => {
        m.emissive.copy(glow);
        m.emissiveIntensity = factor * 1.5;
      });

      controls.update();
      renderer.render(scene, camera);
    };

    animate();

    const onResize = () => {
      if (!container) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };

    window.addEventListener("resize", onResize);
    const timer = setTimeout(onResize, 150);

    return () => {
      cancelAnimationFrame(frameId);
      clearTimeout(timer);
      window.removeEventListener("resize", onResize);
      controls.dispose();
      renderer.dispose();
    };
  }, []);

  return <div ref={containerRef} className="w-full h-full min-h-[400px] bg-transparent" />;
}