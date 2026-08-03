import React, { useEffect, useRef, useState, useCallback } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import robotModel from "./assets/arm_robotics.glb";
import { Bot, Wifi, ChevronDown, Move as MoveIcon, Navigation } from "lucide-react";

// ---------------------------------------------------------------------------
// Design tokens
// ---------------------------------------------------------------------------
const C = {
  bg: "#0a0e1a",
  panel: "#10152a",
  panelAlt: "#0d1224",
  border: "#1c2340",
  borderSoft: "#161c36",
  text: "#e7ebf5",
  sub: "#7480a3",
  subDim: "#4c5578",
  accent: "#3b6cf6",
  accentHover: "#2f57d6",
  accentSoft: "rgba(59,108,246,0.12)",
  green: "#22c55e",
  red: "#ef4444",
  redSoft: "rgba(239,68,68,0.12)",
  track: "#1a2140",
};

const HOME = { j1: 0, j2: 0, j3: 90, j4: 90, j5: 0 };

// ---------------------------------------------------------------------------
// Robot arm link lengths (เมตร — ปรับตามแขนกลจริง)
// L1 = ความสูงฐาน-ไหล่, L2 = แขนบน (ไหล่-ข้อศอก), L3 = แขนล่าง (ข้อศอก-ข้อมือ)
// ---------------------------------------------------------------------------
const L1 = 0.10; // ความสูงไหล่จากพื้น (m)
const L2 = 0.105; // ความยาวแขนบน Upper arm (m)
const L3 = 0.096; // ความยาวแขนล่าง Forearm (m)
const L4 = 0.04; // ระยะจาก J4 (ข้อมือ) ถึง J5 (ปลายมือคีบจริง) — วัดจากโมเดล (m)

/**
 * solveIK — คำนวณมุม Joint จากตำแหน่งปลายมือคีบ (Analytic 2-link planar IK)
 * ตำแหน่ง (x, y, z) ที่รับเข้ามาคือตำแหน่งของ J5 (ปลายมือคีบจริง)
 * โดยจะหักความยาว L4 ออกก่อน แล้วจึงแก้สมการ 2-link ถึงข้อมือ (J4)
 * @param {number} x  - ระยะแกน X จากศูนย์กลางฐาน (m)
 * @param {number} y  - ความสูงจากพื้น (m)
 * @param {number} z  - ระยะแกน Z (depth) จากศูนย์กลางฐาน (m)
 * @returns {{ ok: boolean, j1: number, j2: number, j3: number, j4: number }}
 *   มุมเป็นองศา; ok=false หากตำแหน่งอยู่นอกพิสัย (unreachable)
 */
function solveIK(x, y, z) {
  // J1: หมุนฐานรอบแกน Y — มองจากบน คือ atan2(x, z)
  const j1 = THREE.MathUtils.radToDeg(Math.atan2(x, z));

  // ระยะแนวนอนจากแกนหมุน J2 ถึงปลายมือคีบ (J5) — projection on XZ plane
  const rTotal = Math.sqrt(x * x + z * z);
  // หักความยาวปลายจับ (L4) ออก เพื่อให้เหลือระยะถึงข้อมือ (J4) สำหรับสมการ 2-link
  // (J4 คุมให้ปลายมือคีบชี้แนวนอนเสมอ ส่วนต่อขยาย L4 จึงอยู่ในแนวรัศมีเดียวกัน ไม่กระทบความสูง)
  const r = rTotal - L4;
  // ความสูงจากไหล่ถึงข้อมือ (แนวนอน ไม่เปลี่ยนความสูงจากปลายมือคีบ)
  const dy = y - L1;

  if (r <= 0) {
    return { ok: false, j1, j2: 0, j3: 0, j4: 0 };
  }

  // ระยะตรงจากไหล่ถึงข้อมือ (J4)
  const dist = Math.sqrt(r * r + dy * dy);

  // ตรวจสอบ reachability
  if (dist > L2 + L3 - 0.001) {
    return { ok: false, j1, j2: 0, j3: 0, j4: 0 };
  }
  if (dist < Math.abs(L2 - L3) + 0.001) {
    return { ok: false, j1, j2: 0, j3: 0, j4: 0 };
  }

  // กฎ cosine สำหรับข้อศอก (J3)
  const cosJ3 = (dist * dist - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  const j3Rad = Math.acos(THREE.MathUtils.clamp(cosJ3, -1, 1));

  // J2: มุมไหล่ — alpha (ยกแขน) + beta (มุมภายใน triangle)
  const alpha = Math.atan2(dy, r);
  const sinBeta = (L3 * Math.sin(j3Rad)) / dist;
  const beta = Math.asin(THREE.MathUtils.clamp(sinBeta, -1, 1));
  const j2Rad = alpha + beta;

  // J4: ทำให้ปลายแขนชี้แนวนอน (wrist compensation)
  const j4Rad = -(j2Rad - j3Rad);

  return {
    ok: true,
    j1: THREE.MathUtils.clamp(j1, -180, 180),
    j2: THREE.MathUtils.clamp(THREE.MathUtils.radToDeg(j2Rad), -90, 90),
    j3: THREE.MathUtils.clamp(THREE.MathUtils.radToDeg(j3Rad), -135, 135),
    j4: THREE.MathUtils.clamp(THREE.MathUtils.radToDeg(j4Rad), -135, 135),
  };
}

/**
 * forwardKinematics — คำนวณตำแหน่งปลายมือคีบจริง (J5) จากมุม Joint (FK)
 * ใช้แสดง current end-effector position (รวมระยะ L4 จากข้อมือ J4 ถึงปลายมือคีบ J5)
 */
function forwardKinematics(j1deg, j2deg, j3deg) {
  const j1 = THREE.MathUtils.degToRad(j1deg);
  const j2 = THREE.MathUtils.degToRad(j2deg);
  const j3 = THREE.MathUtils.degToRad(j3deg);

  // ความสูงไหล่
  const shoulderY = L1;
  // ตำแหน่งข้อศอก (ใน plane ที่หมุนตาม J1)
  const elbowR = L2 * Math.cos(j2);
  const elbowY = shoulderY + L2 * Math.sin(j2);
  // ตำแหน่งข้อมือ (J4)
  const wristR = elbowR + L3 * Math.cos(j2 - j3);
  const wristY = elbowY + L3 * Math.sin(j2 - j3);
  // ตำแหน่งปลายมือคีบจริง (J5) — J4 คุมให้ชี้แนวนอนเสมอ จึงบวก L4 ในแนวรัศมีเดียวกัน ความสูงไม่เปลี่ยน
  const gripR = wristR + L4;

  return {
    x: parseFloat((gripR * Math.sin(j1)).toFixed(4)),
    y: parseFloat(wristY.toFixed(4)),
    z: parseFloat((gripR * Math.cos(j1)).toFixed(4)),
  };
}

// J1..J5 limits — ใช้กำหนดขอบเขตของช่องกรอกตำแหน่งเป้าหมาย
const JOINTS = [
  { key: "j1", label: "J1", sub: "ฐาน", min: -180, max: 180, unit: "deg" },
  { key: "j2", label: "J2", sub: "ไหล่", min: -90, max: 90, unit: "deg" },
  { key: "j3", label: "J3", sub: "ข้อศอก", min: -135, max: 135, unit: "deg" },
  { key: "j4", label: "J4", sub: "ข้อมือ", min: -135, max: 135, unit: "deg" },
  { key: "j5", label: "J5", sub: "ปลายจับ", min: 0, max: 100, unit: "%" },
];

// รูปแบบการเคลื่อนที่แบบ CIRA Core / Kinematic motion module
const MOTION_TYPES = [
  { key: "PTP", label: "PTP", desc: "Point-to-Point" },
  { key: "LIN", label: "LIN", desc: "Linear" },
  { key: "CIRC", label: "CIRC", desc: "Circular" },
];

function easeInOutCubic(t) {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

// ---------------------------------------------------------------------------
// 3D Arm Scene — โหลดโมเดล GLTF จาก Blender แล้วอ่าน Joint hierarchy
// จากชื่อ object ที่ตั้งไว้ในไฟล์โมเดล
// ---------------------------------------------------------------------------
function useArmScene(containerRef, joints, wireframe, onJointDelta, onIkDrag) {
  const sceneRef = useRef(null);
  const [modelReady, setModelReady] = useState(false);
  const [modelError, setModelError] = useState(false);
  const onJointDeltaRef = useRef(onJointDelta);
  useEffect(() => { onJointDeltaRef.current = onJointDelta; }, [onJointDelta]);
  const onIkDragRef = useRef(onIkDrag);
  useEffect(() => { onIkDragRef.current = onIkDrag; }, [onIkDrag]);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return undefined;

    // ---- Renderer / Scene / Camera ----
    const scene = new THREE.Scene();

    // Gradient background (dark navy)
    const bgGeo = new THREE.SphereGeometry(28, 24, 16);
    const bgPos = bgGeo.attributes.position;
    const topCol = new THREE.Color(0x060810);
    const botCol = new THREE.Color(0x161d38);
    const bgColors = [];
    for (let i = 0; i < bgPos.count; i++) {
      const y = bgPos.getY(i) / 28;
      const t = THREE.MathUtils.clamp((y + 0.35) / 1.1, 0, 1);
      const c = topCol.clone().lerp(botCol, 1 - t);
      bgColors.push(c.r, c.g, c.b);
    }
    bgGeo.setAttribute("color", new THREE.Float32BufferAttribute(bgColors, 3));
    const bgMesh = new THREE.Mesh(
      bgGeo,
      new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide, fog: false })
    );
    scene.add(bgMesh);

    const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);
    renderer.domElement.style.display = "block";
    renderer.domElement.style.cursor = "grab";

    // ---- Lighting ----
    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    scene.add(new THREE.HemisphereLight(0x8fa4ff, 0x0a0e1a, 0.35));
    const key = new THREE.DirectionalLight(0xffffff, 1.05);
    key.position.set(3.2, 5.5, 3.6);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.camera.near = 1;
    key.shadow.camera.far = 14;
    key.shadow.camera.left = -4;
    key.shadow.camera.right = 4;
    key.shadow.camera.top = 4;
    key.shadow.camera.bottom = -4;
    key.shadow.bias = -0.0015;
    key.shadow.radius = 3;
    scene.add(key);
    const fill = new THREE.DirectionalLight(0xaebfff, 0.28);
    fill.position.set(-3.5, 2.2, -1.5);
    scene.add(fill);
    const rim = new THREE.DirectionalLight(0x3b6cf6, 0.45);
    rim.position.set(-4, 3, -4);
    scene.add(rim);

    // ---- Floor ----
    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(4.4, 48),
      new THREE.ShadowMaterial({ opacity: 0.38 })
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.001;
    floor.receiveShadow = true;
    scene.add(floor);
    const grid = new THREE.GridHelper(8, 32, 0x2c3766, 0x161c36);
    grid.material.opacity = 0.55;
    grid.material.transparent = true;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(0.55));

    // ---- Orbit controller ----
    const controls = {
      azimuth: Math.PI * 0.28,
      elevation: 0.5,
      radius: 4.6,
      target: new THREE.Vector3(0, 0.9, 0),
      dragging: false,
      panMode: false,
      lastX: 0,
      lastY: 0,
    };

    function applyCamera() {
      const el = Math.max(-1.3, Math.min(1.3, controls.elevation));
      const x = controls.target.x + controls.radius * Math.cos(el) * Math.sin(controls.azimuth);
      const y = controls.target.y + controls.radius * Math.sin(el);
      const z = controls.target.z + controls.radius * Math.cos(el) * Math.cos(controls.azimuth);
      camera.position.set(x, y, z);
      camera.lookAt(controls.target);
    }
    applyCamera();

    function resize() {
      const w = container.clientWidth || 1;
      const h = container.clientHeight || 1;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    }
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(container);

    // ---- Gizmo handle drag state ----
    // ลูกศร X/Y/Z (world-aligned, อยู่ที่ปลายแขนเสมอ) -> ลากแล้วคำนวณ IK จริง
    // วงแหวนหมุน (ข้อมือ/ปลายจับ) -> ยังคงขยับ joint นั้นๆ โดยตรงเหมือนเดิม
    const raycaster = new THREE.Raycaster();
    const pointerNDC = new THREE.Vector2();
    const handleDrag = { active: false, mode: null, spec: null, lastX: 0, lastY: 0 };
    const AXIS_VECTORS = {
      x: new THREE.Vector3(1, 0, 0),
      y: new THREE.Vector3(0, 1, 0),
      z: new THREE.Vector3(0, 0, 1),
    };
    const _p0 = new THREE.Vector3();
    const _p1 = new THREE.Vector3();

    // แปลงพิกัดโลก -> พิกัดพิกเซลบนหน้าจอ (สำหรับคำนวณ world-units-per-pixel ของแต่ละแกน)
    function worldToScreen(vec3) {
      const v = vec3.clone().project(camera);
      const rect = renderer.domElement.getBoundingClientRect();
      return { x: (v.x * 0.5 + 0.5) * rect.width, y: (-v.y * 0.5 + 0.5) * rect.height };
    }

    function pickHandle(e) {
      const s = sceneRef.current;
      if (!s || !s.ready || !s.pickables || !s.pickables.length) return null;
      const rect = renderer.domElement.getBoundingClientRect();
      pointerNDC.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      pointerNDC.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointerNDC, camera);
      const hits = raycaster.intersectObjects(s.pickables, false);
      return hits.length ? hits[0].object.userData : null;
    }

    function onPointerDown(e) {
      const spec = pickHandle(e);
      if (spec && spec.axis && onIkDragRef.current) {
        // ลากลูกศร -> โหมด IK translate
        handleDrag.active = true;
        handleDrag.mode = "ik";
        handleDrag.spec = spec;
        handleDrag.lastX = e.clientX;
        handleDrag.lastY = e.clientY;
        renderer.domElement.style.cursor = "grabbing";
        return;
      }
      controls.dragging = true;
      controls.panMode = e.shiftKey || controls.panMode;
      controls.lastX = e.clientX;
      controls.lastY = e.clientY;
      renderer.domElement.style.cursor = "grabbing";
    }
    function onPointerMove(e) {
      if (handleDrag.active && handleDrag.mode === "ik") {
        const s = sceneRef.current;
        const dx = e.clientX - handleDrag.lastX;
        const dy = e.clientY - handleDrag.lastY;
        handleDrag.lastX = e.clientX;
        handleDrag.lastY = e.clientY;
        if (s && s.translateGizmo) {
          const axis = handleDrag.spec.axis;
          const gizmoPos = s.translateGizmo.position;
          // ใช้พิกัดฉาก (screen projection) ของแกนนั้นแค่หา "ทิศทาง" บนจอเท่านั้น
          // (ระยะจริงในหน่วยเมตรของ IK เป็นคนละสเกลกับหน่วยฉากในโมเดล 3D ที่ถูกขยาย 5 เท่า
          //  จึงแปลงระยะพิกเซลเป็นเมตรด้วยค่าความไวคงที่แทน ไม่ผูกกับสเกลภาพ)
          _p0.copy(gizmoPos);
          _p1.copy(gizmoPos).addScaledVector(AXIS_VECTORS[axis], 0.05);
          const sp0 = worldToScreen(_p0);
          const sp1 = worldToScreen(_p1);
          let sdx = sp1.x - sp0.x;
          let sdy = sp1.y - sp0.y;
          const slen = Math.hypot(sdx, sdy) || 1e-6;
          sdx /= slen;
          sdy /= slen;
          const movedAlongAxisPixels = dx * sdx + dy * sdy;
          const IK_METERS_PER_PIXEL = 0.0006; // ปรับความไวการลากได้ตรงนี้
          const worldDelta = movedAlongAxisPixels * IK_METERS_PER_PIXEL;
          onIkDragRef.current?.(axis, worldDelta);
        }
        return;
      }
      if (!controls.dragging) return;
      const dx = e.clientX - controls.lastX;
      const dy = e.clientY - controls.lastY;
      controls.lastX = e.clientX;
      controls.lastY = e.clientY;
      if (controls.panMode) {
        controls.target.x -= dx * 0.004;
        controls.target.y += dy * 0.004;
      } else {
        controls.azimuth -= dx * 0.006;
        controls.elevation += dy * 0.006;
      }
    }
    function onPointerUp() {
      handleDrag.active = false;
      handleDrag.mode = null;
      handleDrag.spec = null;
      controls.dragging = false;
      renderer.domElement.style.cursor = "grab";
    }
    function onWheel(e) {
      e.preventDefault();
      controls.radius = Math.max(2, Math.min(10, controls.radius + e.deltaY * 0.0025));
    }
    renderer.domElement.addEventListener("pointerdown", onPointerDown);
    window.addEventListener("pointermove", onPointerMove);
    window.addEventListener("pointerup", onPointerUp);
    renderer.domElement.addEventListener("wheel", onWheel, { passive: false });

    const _eeWorldPos = new THREE.Vector3();
    let raf;
    function tick() {
      applyCamera();
      const s = sceneRef.current;
      if (s && s.ready && s.endEffector && s.translateGizmo) {
        s.endEffector.getWorldPosition(_eeWorldPos);
        s.translateGizmo.position.copy(_eeWorldPos);
      }
      renderer.render(scene, camera);
      raf = requestAnimationFrame(tick);
    }
    tick();

    sceneRef.current = {
      scene, camera, renderer, controls,
      baseGroup: null,
      shoulder: null,
      elbow: null,
      wrist: null,
      gripperGroup: null,
      fingerL: null,
      fingerR: null,
      endEffector: null,
      allMeshes: [],
      pickables: [],
      translateGizmo: null,
      rotateGizmo: null,
      ready: false,
    };

    // ---- โหลดโมเดล GLTF ----
    const loader = new GLTFLoader();

    loader.load(
      robotModel,
      (gltf) => {
        buildFromGLTF(gltf);
      },
      undefined,
      (err) => {
        console.error("GLTF ERROR", err);
        setModelError(true);
      }
    );

    function buildFromGLTF(gltf) {
      const s = sceneRef.current;
      if (!s) return;

      const model = gltf.scene;
      scene.add(model);

      model.scale.set(5, 5, 5);

      const box = new THREE.Box3().setFromObject(model);
      const minY = box.min.y;

      // ยกโมเดลขึ้นให้ฐานแตะพื้น
      model.position.y -= minY;

      s.baseGroup = model.getObjectByName("Gear_for_Base");
      s.shoulder = model.getObjectByName("ArmJ2");
      s.elbow = model.getObjectByName("ArmJ3");
      s.wrist = model.getObjectByName("ArmGriper");
      s.gripperGroup = model.getObjectByName("FingerBase");
      s.fingerL = model.getObjectByName("Left_Fringer");
      s.fingerR = model.getObjectByName("Right_Finger");

      if (s.fingerL) s.fingerLHome = s.fingerL.position.clone();
      if (s.fingerR) s.fingerRHome = s.fingerR.position.clone();

      // Create J5 end-effector pivot at gripper tip
      const endEffector = new THREE.Object3D();
      endEffector.name = "J5_EndEffector";
      endEffector.position.set(0, -L4, 0);
      s.gripperGroup.add(endEffector);
      s.endEffector = endEffector;

      // ---- Gizmo ที่ปลายมือคีบ (J5) ----
      // ลูกศรเลื่อน XYZ เท่านั้น (world-aligned, ไม่หมุนตามข้อต่อ)
      // อยู่ที่ "ปลายมือคีบจริง" (จุดกึ่งกลางระหว่างนิ้วซ้าย-ขวา คือตำแหน่ง J5)
      // ลากลูกศรแกนไหน -> ปลายมือคีบขยับไปทางแกนนั้นในพิกัดโลกจริง แล้วคำนวณ IK
      // ย้อนกลับไปหามุมของทุกข้อต่อ (J1-J4) ให้ปลายมือคีบไปถึงตำแหน่งนั้น (แดง=X เขียว=Y น้ำเงิน=Z)
      const gizmoLen = 0.2;

      function makeArrow(dir, color, axis) {
        const shaftLen = gizmoLen * 0.7;
        const headLen = gizmoLen * 0.3;
        const mat = new THREE.MeshBasicMaterial({ color, depthTest: false });
        const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, shaftLen, 8), mat);
        shaft.position.y = shaftLen / 2;
        const head = new THREE.Mesh(new THREE.ConeGeometry(0.018, headLen, 10), mat);
        head.position.y = shaftLen + headLen / 2;
        const grp = new THREE.Group();
        grp.add(shaft, head);
        grp.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
        grp.renderOrder = 999;
        const userData = { axis };
        shaft.userData = userData;
        head.userData = userData;
        return { grp, pickables: [shaft, head] };
      }

      s.pickables = [];

      // ---- ลูกศรเลื่อน XYZ (world space) -> ผูกกับ IK จริง ----
      const translateGizmo = new THREE.Group();
      translateGizmo.name = "IK_TranslateGizmo";
      const arrows = [
        makeArrow(new THREE.Vector3(1, 0, 0), 0xef4444, "x"),
        makeArrow(new THREE.Vector3(0, 1, 0), 0x22c55e, "y"),
        makeArrow(new THREE.Vector3(0, 0, 1), 0x3b6cf6, "z"),
      ];
      arrows.forEach(({ grp, pickables }) => {
        translateGizmo.add(grp);
        s.pickables.push(...pickables);
      });
      // เพิ่มเข้า scene โดยตรง (ไม่ใช่ลูกของ endEffector) เพื่อให้แกนอ้างอิงกับโลกเสมอ
      // ไม่หมุนตามข้อต่อ — ตำแหน่งจะถูกอัปเดตให้ตรงกับ "ปลายมือคีบ (J5)" ทุกเฟรมใน tick()
      scene.add(translateGizmo);
      s.translateGizmo = translateGizmo;

      const missing = ["baseGroup", "shoulder", "elbow", "wrist", "gripperGroup", "fingerL", "fingerR"]
        .filter((k) => !s[k]);
      if (missing.length) {
        console.error("GLTF model is missing expected named objects:", missing);
        setModelError(true);
        return;
      }

      s.allMeshes = [];
      model.traverse((o) => {
        if (o.isMesh) {
          o.castShadow = true;
          o.receiveShadow = true;
          s.allMeshes.push(o);
        }
      });

      s.ready = true;
      setModelReady(true);
    }

    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      renderer.domElement.removeEventListener("pointerdown", onPointerDown);
      window.removeEventListener("pointermove", onPointerMove);
      window.removeEventListener("pointerup", onPointerUp);
      renderer.domElement.removeEventListener("wheel", onWheel);
      renderer.dispose();
      if (container.contains(renderer.domElement)) container.removeChild(renderer.domElement);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    const s = sceneRef.current;
    if (!s || !s.ready) return;
    const d = THREE.MathUtils.degToRad;

    s.baseGroup.rotation.y = d(joints.j1);
    s.shoulder.rotation.x = d(-joints.j2);
    s.elbow.rotation.y = d(-joints.j3);
    s.wrist.rotation.y = d(-joints.j4);

    const spread = (joints.j5 / 100) * 0.01;
    s.fingerL.position.copy(s.fingerLHome);
    s.fingerR.position.copy(s.fingerRHome);
    s.fingerL.translateY(spread);
    s.fingerR.translateY(-spread);

    s.baseGroup.updateMatrixWorld(true);
  }, [joints, modelReady]);

  useEffect(() => {
    const s = sceneRef.current;
    if (!s || !s.ready) return;
    s.allMeshes.forEach((m) => { m.material.wireframe = wireframe; });
  }, [wireframe, modelReady]);

  return { modelReady, modelError };
}

// ---------------------------------------------------------------------------
// Main component — Header (title + port select + connect) + 3D Arm viewer
// ---------------------------------------------------------------------------
export default function RoboticArmControl() {
  const [ports, setPorts] = useState(["COM3"]);
  const [selectedPort, setSelectedPort] = useState("COM3");
  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);

  // ---- Kinematic motion module (CIRA Core style: PTP / LIN / CIRC) ----
  const [joints, setJoints] = useState(HOME);
  const [targets, setTargets] = useState(HOME);
  const [motionType, setMotionType] = useState("PTP");
  const [moving, setMoving] = useState(false);
  const animRef = useRef(null);
  const jointsRef = useRef(HOME);
  useEffect(() => { jointsRef.current = joints; }, [joints]);

  // ---- Inverse Kinematics panel state ----
  const ikHome = forwardKinematics(HOME.j1, HOME.j2, HOME.j3);
  const [ikTarget, setIkTarget] = useState({ x: ikHome.x, y: ikHome.y, z: ikHome.z });
  const ikTargetRef = useRef(ikTarget); // แหล่งข้อมูลจริงแบบ sync ใช้ตอนลากกิซโมต่อเนื่อง
  const [ikError, setIkError] = useState("");
  const [ikMode, setIkMode] = useState(false); // toggle IK / Joint input panel

  // FK readout — อัปเดตทุกครั้งที่ joint เปลี่ยน
  const fkPos = forwardKinematics(joints.j1, joints.j2, joints.j3);

  // ---- ตัวช่วย animate จากมุม A ไป B (ใช้ร่วมกันทั้ง Joint move และ IK move) ----
  // ทุกครั้งที่เล่น (step ที่สอง) จะอัปเดต jointsRef ให้ตรงกับ state เสมอ
  // เพื่อให้ปุ่ม Move ครั้งถัดไป "รู้" ว่าแขนอยู่ตรงไหนจริง ๆ
  const animateJoints = useCallback(({ from, to, duration, motion, onDone }) => {
    const start = performance.now();
    function step(now) {
      const t = Math.min(1, (now - start) / duration);
      const e = motion === "LIN" ? t : easeInOutCubic(t);
      const next = {};
      JOINTS.forEach((j) => {
        let val = from[j.key] + (to[j.key] - from[j.key]) * e;
        if (motion === "CIRC" && (j.key === "j2" || j.key === "j3")) {
          val += Math.sin(t * Math.PI) * 8 * (j.key === "j2" ? 1 : -1);
        }
        next[j.key] = val;
      });
      setJoints(next);
      jointsRef.current = next;
      if (t < 1) {
        animRef.current = requestAnimationFrame(step);
      } else {
        setJoints(to);
        jointsRef.current = to;
        onDone?.();
      }
    }
    animRef.current = requestAnimationFrame(step);
  }, []);

  // ---- Reset กลับตำแหน่งเริ่มต้นก่อนเสมอ แล้วค่อยเคลื่อนไปยังเป้าหมาย ----
  // ไม่ว่าปลายมือคีบ (end-effector) จะอยู่ตำแหน่งใดก็ตามก่อนกดปุ่ม
  // เมื่อกด Move ทุกข้อต่อจะเคลื่อนกลับไปที่ค่าองศาเริ่มต้น (HOME) ก่อน
  // แล้วจึงคำนวณ/เคลื่อนที่ต่อไปยังตำแหน่งเป้าหมายจริง — ทำให้ทุก Move
  // เริ่มต้นจากองศาเดิมเหมือนกันเสมอ (ตาม CiRA CORE Home + Plan & Execute)
  const HOME_RESET_MS = 700;
  const runFromHome = useCallback((to, motion, onFinalDone) => {
    setMoving(true);
    const fromNow = { ...jointsRef.current };
    animateJoints({
      from: fromNow,
      to: HOME,
      duration: HOME_RESET_MS,
      motion: "PTP",
      onDone: () => {
        const duration = motion === "LIN" ? 1400 : motion === "CIRC" ? 1800 : 1100;
        animateJoints({
          from: HOME,
          to,
          duration,
          motion,
          onDone: () => {
            setMoving(false);
            onFinalDone?.();
          },
        });
      },
    });
  }, [animateJoints]);

  const handleIkMove = useCallback(() => {
    if (moving) return;
    const { x, y, z } = ikTarget;
    const result = solveIK(parseFloat(x) || 0, parseFloat(y) || 0, parseFloat(z) || 0);
    if (!result.ok) {
      setIkError("ตำแหน่งอยู่นอกพิสัยของแขนกล (Unreachable)");
      return;
    }
    setIkError("");
    // อัปเดต targets แล้วส่งต่อให้ animation ทำงาน
    const newTargets = {
      j1: result.j1,
      j2: result.j2,
      j3: result.j3,
      j4: result.j4,
      j5: jointsRef.current.j5,
    };
    setTargets(newTargets);

    // เริ่มจากองศาเริ่มต้น (HOME) เสมอ ก่อนเคลื่อนไปยังเป้าหมายที่คำนวณจาก IK
    runFromHome(newTargets, motionType, () => {
      ikTargetRef.current = { x, y, z };
    });
  }, [moving, motionType, ikTarget, runFromHome]);

  // ---- ลากลูกศร XYZ ที่ปลายแขน -> ขยับตำแหน่งจริงแบบเรียลไทม์แล้วคำนวณ IK ย้อนกลับ ----
  // ทุกข้อต่อ (J1-J4) จะถูกคำนวณใหม่ให้ปลายแขนไปอยู่ที่ตำแหน่ง XYZ เป้าหมายเสมอ
  const handleIkDrag = useCallback((axis, delta) => {
    if (moving) return;
    const cur = ikTargetRef.current;
    const next = { ...cur, [axis]: (parseFloat(cur[axis]) || 0) + delta };
    const result = solveIK(next.x, next.y, next.z);
    if (!result.ok) {
      // นอกพิสัย — ค้างตำแหน่งเดิมไว้ (ไม่ขยับต่อในทิศทางนั้น)
      return;
    }
    ikTargetRef.current = next;
    setIkTarget(next);
    setIkError("");
    const newJoints = {
      j1: result.j1,
      j2: result.j2,
      j3: result.j3,
      j4: result.j4,
      j5: jointsRef.current.j5,
    };
    jointsRef.current = newJoints;
    setJoints(newJoints);
    setTargets(newJoints);
  }, [moving]);

  const handleJointDelta = useCallback((key, delta) => {
    if (moving) return; // อย่าให้ลากพร้อมกับตอนที่ Move กำลังเล่น animation
    const def = JOINTS.find((j) => j.key === key);
    if (!def) return;
    setJoints((prev) => {
      const v = Math.max(def.min, Math.min(def.max, prev[key] + delta));
      const next = { ...prev, [key]: v };
      jointsRef.current = next;
      return next;
    });
    setTargets((prev) => {
      const base = prev[key] === "" || prev[key] === undefined ? jointsRef.current[key] : prev[key];
      const v = Math.max(def.min, Math.min(def.max, base + delta));
      return { ...prev, [key]: v };
    });
  }, [moving]);

  const viewerRef = useRef(null);
  const { modelReady, modelError } = useArmScene(viewerRef, joints, false, handleJointDelta, handleIkDrag);

  const handleTargetChange = (key, min, max, raw) => {
    const v = raw === "" ? "" : Math.max(min, Math.min(max, parseFloat(raw)));
    setTargets((t) => ({ ...t, [key]: raw === "" ? "" : v }));
  };

  const handleMove = useCallback(() => {
    if (moving) return;
    const to = { ...jointsRef.current };
    JOINTS.forEach((j) => {
      const v = targets[j.key];
      to[j.key] = v === "" || v === undefined || Number.isNaN(v) ? jointsRef.current[j.key] : v;
    });

    // เริ่มจากองศาเริ่มต้น (HOME) เสมอ ก่อนเคลื่อนไปยังเป้าหมายที่กรอกไว้
    runFromHome(to, motionType, () => {
      const fk = forwardKinematics(to.j1, to.j2, to.j3);
      ikTargetRef.current = fk;
      setIkTarget(fk);
    });
  }, [moving, motionType, targets, runFromHome]);

  useEffect(() => () => { if (animRef.current) cancelAnimationFrame(animRef.current); }, []);

  // ถ้ารันในแอป Electron จะมี window.electronAPI ให้ใช้จริง
  const refreshPorts = useCallback(async () => {
    if (!window.electronAPI?.listPorts) return;
    try {
      const list = await window.electronAPI.listPorts();
      setPorts(list);
      if (list.length === 0) {
        setSelectedPort("");
        setConnected(false);
        return;
      }
      if (!list.includes(selectedPort)) {
        setSelectedPort(list[0]);
      }
    } catch (err) {
      console.error(err);
    }
  }, [selectedPort]);

  useEffect(() => {
    refreshPorts();
  }, [refreshPorts]);

  const handleConnect = useCallback(async () => {
    if (!selectedPort) return;
    if (!window.electronAPI?.connectPort) {
      setConnected(false);
      return;
    }
    setConnecting(true);
    try {
      const res = await window.electronAPI.connectPort(selectedPort);
      setConnected(!!res?.ok);
    } catch {
      setConnected(false);
    } finally {
      setConnecting(false);
    }
  }, [selectedPort]);

  return (
    <div
      className="w-full h-full min-h-screen flex flex-col"
      style={{ background: C.bg, fontFamily: "'IBM Plex Sans Thai', 'Inter', sans-serif" }}
    >
      {/* ---------------- Header ---------------- */}
      <div
        className="flex items-center justify-between px-5 py-3 shrink-0"
        style={{ background: C.panel, borderBottom: `1px solid ${C.border}` }}
      >
        <div className="flex items-center gap-2.5">
          <div
            className="w-8 h-8 rounded-lg flex items-center justify-center"
            style={{ background: C.accentSoft }}
          >
            <Bot size={18} color={C.accent} />
          </div>
          <span className="text-[15px] font-semibold" style={{ color: C.text }}>
            ANR Robot Studio
          </span>
        </div>

        <div className="flex items-center gap-3">
          <div
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs relative"
            style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text }}
          >
            <Wifi size={13} color={C.sub} />
            <select
              value={selectedPort}
              onChange={(e) => setSelectedPort(e.target.value)}
              className="bg-transparent outline-none appearance-none pr-4"
              style={{ color: C.text }}
            >
              {ports.map((p) => (
                <option key={p} value={p} style={{ background: C.panel }}>{p}</option>
              ))}
            </select>
            <ChevronDown size={13} color={C.subDim} className="pointer-events-none absolute right-2.5" />
          </div>

          <button
            onClick={handleConnect}
            disabled={connected || connecting || !selectedPort}
            className="flex items-center gap-2 px-4 py-1.5 rounded-full text-xs font-medium transition-colors"
            style={{
              background: connected ? "rgba(34,197,94,0.12)" : C.accent,
              color: connected ? C.green : "#fff",
              opacity: connecting ? 0.7 : 1,
              cursor: connected || connecting || !selectedPort ? "default" : "pointer",
            }}
          >
            <span
              className="w-1.5 h-1.5 rounded-full"
              style={{ background: connected ? C.green : "#fff" }}
            />
            {connecting ? "กำลังเชื่อมต่อ..." : connected ? "เชื่อมต่อแล้ว" : "Connect"}
          </button>
        </div>
      </div>

      {/* ---------------- 3D Robot Arm Viewer ---------------- */}
      <div className="flex-1 min-h-0 p-4">
        <div
          className="w-full h-full rounded-2xl overflow-hidden relative"
          style={{ background: C.panel, border: `1px solid ${C.border}` }}
        >
          <div ref={viewerRef} className="w-full h-full" />
          {!modelReady && !modelError && (
            <div
              className="absolute inset-0 flex items-center justify-center text-sm"
              style={{ color: C.sub }}
            >
              กำลังโหลดโมเดล 3D...
            </div>
          )}
          {modelError && (
            <div
              className="absolute inset-0 flex items-center justify-center text-sm"
              style={{ color: C.red }}
            >
              โหลดโมเดล 3D ไม่สำเร็จ
            </div>
          )}

          {/* ---------------- Kinematic Move panel (IK / Joint tabs) ---------------- */}
          <div
            className="absolute top-4 right-4 w-68 rounded-2xl overflow-hidden shadow-2xl"
            style={{ width: 272, background: "rgba(16,21,42,0.92)", border: `1px solid ${C.border}`, backdropFilter: "blur(6px)" }}
          >
            {/* Header + tab switcher */}
            <div className="flex items-center justify-between px-4 pt-3.5 pb-2.5" style={{ borderBottom: `1px solid ${C.borderSoft}` }}>
              <div className="flex items-center gap-2">
                <Navigation size={14} color={C.accent} />
                <span className="text-xs font-semibold" style={{ color: C.text }}>Kinematic Move</span>
              </div>
              <div className="flex rounded-lg overflow-hidden" style={{ border: `1px solid ${C.borderSoft}` }}>
                {[{ key: false, label: "Joint" }, { key: true, label: "IK (XYZ)" }].map(({ key, label }) => (
                  <button
                    key={String(key)}
                    onClick={() => { setIkMode(key); setIkError(""); }}
                    className="px-2.5 py-1 text-[10px] font-semibold transition-colors"
                    style={{
                      background: ikMode === key ? C.accent : C.panelAlt,
                      color: ikMode === key ? "#fff" : C.sub,
                    }}
                  >
                    {label}
                  </button>
                ))}
              </div>
            </div>

            {/* Motion type selector */}
            <div className="px-4 pt-3">
              <div className="text-[10px] uppercase tracking-wide mb-1.5" style={{ color: C.subDim }}>Motion Type</div>
              <div className="grid grid-cols-3 gap-1.5">
                {MOTION_TYPES.map((m) => {
                  const active = motionType === m.key;
                  return (
                    <button
                      key={m.key}
                      onClick={() => setMotionType(m.key)}
                      title={m.desc}
                      className="rounded-lg py-1.5 text-[11px] font-semibold transition-colors"
                      style={{
                        background: active ? C.accent : C.panelAlt,
                        color: active ? "#fff" : C.sub,
                        border: `1px solid ${active ? C.accent : C.borderSoft}`,
                      }}
                    >
                      {m.label}
                    </button>
                  );
                })}
              </div>
            </div>

            {ikMode ? (
              /* ===== IK (XYZ) tab ===== */
              <div className="px-4 pt-3.5 flex flex-col gap-2.5">
                {/* FK readout — ตำแหน่งปัจจุบันของปลายแขน */}
                <div className="rounded-xl px-3 py-2.5" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}>
                  <div className="text-[10px] uppercase tracking-wide mb-1.5" style={{ color: C.subDim }}>
                    📍 ตำแหน่งปัจจุบัน (FK)
                  </div>
                  <div className="grid grid-cols-3 gap-1.5">
                    {[["X", fkPos.x], ["Y", fkPos.y], ["Z", fkPos.z]].map(([axis, val]) => (
                      <div key={axis} className="text-center">
                        <div className="text-[10px]" style={{ color: C.subDim }}>{axis}</div>
                        <div className="text-[12px] font-mono" style={{ color: C.accent }}>{val.toFixed(3)}</div>
                      </div>
                    ))}
                  </div>
                  <div className="text-[9px] mt-1 text-center" style={{ color: C.subDim }}>หน่วย: เมตร (m)</div>
                </div>

                {/* IK target inputs */}
                <div>
                  <div className="text-[10px] uppercase tracking-wide mb-1.5" style={{ color: C.subDim }}>
                    🎯 ตำแหน่งเป้าหมาย (m)
                  </div>
                  {[
                    { axis: "x", label: "X", hint: "ซ้าย ↔ ขวา", color: "#ef4444" },
                    { axis: "y", label: "Y", hint: "ลง ↕ ขึ้น", color: "#22c55e" },
                    { axis: "z", label: "Z", hint: "หน้า ↔ หลัง", color: "#3b6cf6" },
                  ].map(({ axis, label, hint, color }) => (
                    <div key={axis} className="flex items-center justify-between gap-2 mb-1.5">
                      <div className="w-16 shrink-0">
                        <span className="text-[12px] font-bold" style={{ color }}>{label}</span>
                        <span className="text-[9px] block" style={{ color: C.subDim }}>{hint}</span>
                      </div>
                      <input
                        type="number"
                        step="0.001"
                        value={ikTarget[axis]}
                        onChange={(e) => {
                          setIkTarget((prev) => ({ ...prev, [axis]: e.target.value }));
                          setIkError("");
                        }}
                        disabled={moving}
                        className="flex-1 min-w-0 rounded-lg px-2 py-1.5 text-[12px] font-mono text-right outline-none"
                        style={{ background: C.track, border: `1px solid ${C.borderSoft}`, color: C.text, opacity: moving ? 0.5 : 1 }}
                      />
                    </div>
                  ))}
                </div>

                {/* IK error message */}
                {ikError && (
                  <div className="rounded-lg px-3 py-2 text-[11px]" style={{ background: C.redSoft, border: `1px solid ${C.red}`, color: C.red }}>
                    ⚠️ {ikError}
                  </div>
                )}

                {/* Reachability hint */}
                <div className="text-[10px] rounded-lg px-3 py-2" style={{ background: C.accentSoft, color: C.sub }}>
                  พิสัยสูงสุด ≈ {(L2 + L3).toFixed(3)} m จากแกนหมุน J1
                  <br/>ความสูงไหล่ L1 = {L1.toFixed(3)} m
                </div>

                {/* Drag-gizmo hint */}
                <div className="text-[10px] rounded-lg px-3 py-2" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.sub }}>
                  🖱️ หรือลากลูกศร <span style={{ color: "#ef4444" }}>X</span>/<span style={{ color: "#22c55e" }}>Y</span>/<span style={{ color: "#3b6cf6" }}>Z</span> ที่ปลายแขนในโมเดล 3D โดยตรง — ทุกข้อต่อจะคำนวณ IK ตามแบบเรียลไทม์
                </div>
              </div>
            ) : (
              /* ===== Joint tab (เดิม) ===== */
              <div className="px-4 pt-3.5 flex flex-col gap-2">
                {JOINTS.map((j) => (
                  <div key={j.key} className="flex items-center justify-between gap-2">
                    <span className="text-[11px] w-16 shrink-0" style={{ color: C.sub }}>
                      {j.label} <span style={{ color: C.subDim }}>({j.unit === "%" ? "%" : "°"})</span>
                    </span>
                    <input
                      type="number"
                      value={targets[j.key]}
                      min={j.min}
                      max={j.max}
                      onChange={(e) => handleTargetChange(j.key, j.min, j.max, e.target.value)}
                      disabled={moving}
                      className="flex-1 min-w-0 rounded-lg px-2 py-1 text-[12px] font-mono text-right outline-none"
                      style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text, opacity: moving ? 0.5 : 1 }}
                    />
                  </div>
                ))}
              </div>
            )}

            {/* Move button */}
            <div className="px-4 py-3.5">
              <button
                onClick={ikMode ? handleIkMove : handleMove}
                disabled={moving || !modelReady}
                className="w-full flex items-center justify-center gap-2 rounded-xl py-2.5 text-xs font-semibold transition-colors"
                style={{
                  background: moving ? C.panelAlt : C.accent,
                  color: moving ? C.sub : "#fff",
                  border: `1px solid ${moving ? C.borderSoft : C.accent}`,
                  cursor: moving || !modelReady ? "default" : "pointer",
                }}
              >
                <MoveIcon size={14} />
                {moving
                  ? `กำลังเคลื่อนที่ (${motionType})...`
                  : ikMode
                    ? `IK Move (${motionType})`
                    : `Move (${motionType})`}
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
