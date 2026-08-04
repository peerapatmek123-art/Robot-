import React, { useEffect, useRef, useState, useCallback } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import robotModel from "./assets/arm_robotics.glb";
import { Bot, Wifi, ChevronDown, Move as MoveIcon, Navigation, Home as HomeIcon, Save, Trash2, PlayCircle, BookMarked } from "lucide-react";

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
// L4 = ระยะจาก J4 (ข้อมือ) ถึง J5 (ปลายมือคีบจริง)
// ค่าเริ่มต้นนี้เป็นแค่ fallback ก่อนโมเดลโหลดเสร็จ — หลังโหลดเสร็จ useArmScene
// จะ "คาลิเบรต" ค่านี้ใหม่จากตำแหน่งจริงของ Left_Fringer/Right_Finger ในโมเดล (ไม่เดาอีกต่อไป)
let L4 = 0.04;

/**
 * solveIK — คำนวณมุม Joint จากตำแหน่งปลายมือคีบ (Analytic 2-link planar IK)
 * ตำแหน่ง (x, y, z) ที่รับเข้ามาคือตำแหน่งของ J5 (ปลายมือคีบจริง)
 * โดยจะหักความยาว L4 ออกก่อน แล้วจึงแก้สมการ 2-link ถึงข้อมือ (J4)
 *
 * ข้อศอก (J3) มี 2 configuration ที่ไปถึงตำแหน่งเดียวกันได้ (งอ "บวก" หรือ "ลบ")
 * โดยเฉพาะจุดต่ำใกล้ฐาน มักไปถึงได้จริงแค่ configuration เดียวภายใต้ขอบเขตมุมของข้อต่อ
 * ฟังก์ชันนี้จึงคำนวณทั้งสองแบบแล้วเลือกแบบที่อยู่ในขอบเขตจริงของ J2/J3/J4
 * (เดิมใช้แบบเดียวแล้ว clamp ทิ้ง ทำให้บางจุดที่ไปถึงได้จริงถูกปัดเป็นตำแหน่งผิดเงียบ ๆ)
 * @param {number} x  - ระยะแกน X จากศูนย์กลางฐาน (m)
 * @param {number} y  - ความสูงจากพื้น (m)
 * @param {number} z  - ระยะแกน Z (depth) จากศูนย์กลางฐาน (m)
 * @returns {{ ok: boolean, j1: number, j2: number, j3: number, j4: number }}
 *   มุมเป็นองศา; ok=false หากตำแหน่งอยู่นอกพิสัย (unreachable)
 */
function solveIK(x, y, z, prevJoints) {
  // J1: หมุนฐานรอบแกน Y — มองจากบน คือ atan2(x, z)
  const j1 = THREE.MathUtils.radToDeg(Math.atan2(x, z));

  // ระยะแนวนอนจากแกนหมุน J2 ถึงปลายมือคีบ (J5) — projection on XZ plane
  const rTotal = Math.sqrt(x * x + z * z);
  // หักความยาวปลายจับ (L4) ออก เพื่อให้เหลือระยะถึงข้อมือ (J4) สำหรับสมการ 2-link
  // (J4 คุมให้ปลายมือคีบชี้แนวนอนเสมอ ส่วนต่อขยาย L4 จึงอยู่ในแนวรัศมีเดียวกัน ไม่กระทบความสูง)
  const r = rTotal - L4;
  // ความสูงจากไหล่ถึงข้อมือ (แนวนอน ไม่เปลี่ยนความสูงจากปลายมือคีบ)
  const dy = y - L1;

  // ปฏิเสธเป้าหมายที่ใกล้แกนหมุนฐานเกินไป (r เล็กมาก) — โซนนี้เป็น near-singularity ที่
  // เงื่อนไข "บังคับมือคีบให้อยู่แนวระดับเสมอ" ต้องพับข้อศอก/ข้อมือสุดขั้วเพื่อสนอง dy
  // ทั้งที่ระยะแนวรัศมีแทบไม่เหลือ ทำให้ได้ท่าที่พับซ้อนกันจนดูหักงอผิดธรรมชาติ (เหมือนภาพที่ 4)
  // ตัดขอบเขตขั้นต่ำไว้กันไม่ให้คำนวณเข้าโซนนี้เลย ถือว่านอกพิสัยแทน
  const MIN_REACH = 0.035; // เมตร
  if (r <= MIN_REACH) {
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

  // กฎ cosine สำหรับข้อศอก (J3) — j3Mag คือขนาดมุมงอ (เสมอค่าบวก, [0, π])
  const cosJ3 = (dist * dist - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  const j3Mag = Math.acos(THREE.MathUtils.clamp(cosJ3, -1, 1));

  // alpha: มุมยกแขนจาก J2 ไปยัง J4 ตามแนวเป้าหมาย, betaMag: มุมภายใน triangle
  const alpha = Math.atan2(dy, r);
  const sinBetaMag = THREE.MathUtils.clamp((L3 * Math.sin(j3Mag)) / dist, -1, 1);
  const betaMag = Math.asin(sinBetaMag);

  // ---- Configuration A: ศอกงอ "บวก" (ตามสูตรเดิม) ----
  const j3RadA = j3Mag;
  const j2RadA = alpha + betaMag;
  const j4RadA = -(j2RadA - j3RadA);

  // ---- Configuration B: ศอกงอ "ลบ" (มิเรอร์ — ช่วยให้ไปถึงจุดต่ำ/ใกล้ฐานที่ A ไปไม่ถึง) ----
  const j3RadB = -j3Mag;
  const j2RadB = alpha - betaMag;
  const j4RadB = -(j2RadB - j3RadB);

  const toDeg = (rad) => THREE.MathUtils.radToDeg(rad);
  const candidates = [
    { j2: toDeg(j2RadA), j3: toDeg(j3RadA), j4: toDeg(j4RadA) },
    { j2: toDeg(j2RadB), j3: toDeg(j3RadB), j4: toDeg(j4RadB) },
  ];

  // เผื่อ margin เล็กน้อย (0.01°) กันเคสอยู่ติดขอบพอดีถูกตัดทิ้งเพราะ floating point
  const inRange = (v, lo, hi) => v >= lo - 0.01 && v <= hi + 0.01;
  const validOnes = candidates.filter(
    (c) => inRange(c.j2, -90, 90) && inRange(c.j3, -135, 135) && inRange(c.j4, -135, 135)
  );

  let chosen;
  if (validOnes.length === 0) {
    // ไม่มี configuration ไหนอยู่ในขอบเขตพอดี ใช้ตัวแรก (พฤติกรรมเดิม) แล้ว clamp ต่อไป
    chosen = candidates[0];
  } else if (validOnes.length === 1 || !prevJoints) {
    chosen = validOnes[0];
  } else {
    // มีทั้ง 2 configuration ที่ไปถึงได้ — เลือกอันที่ "ใกล้ท่าปัจจุบัน" ที่สุดในปริภูมิมุมข้อต่อ
    // (แทนที่จะเลือกตัวแรกที่เจอแบบสุ่ม) กันไม่ให้ข้อศอก/ไหล่สลับ configuration กะทันหัน
    // ระหว่างลาก ซึ่งเป็นสาเหตุหลักที่ทำให้ท่าทางดูกระโดด/ไม่ต่อเนื่อง/ผิดธรรมชาติ
    const jointDist = (c) =>
      Math.abs(c.j2 - prevJoints.j2) + Math.abs(c.j3 - prevJoints.j3) + Math.abs(c.j4 - prevJoints.j4);
    chosen = jointDist(validOnes[0]) <= jointDist(validOnes[1]) ? validOnes[0] : validOnes[1];
  }

  return {
    ok: true,
    j1: THREE.MathUtils.clamp(j1, -180, 180),
    j2: THREE.MathUtils.clamp(chosen.j2, -90, 90),
    j3: THREE.MathUtils.clamp(chosen.j3, -135, 135),
    j4: THREE.MathUtils.clamp(chosen.j4, -135, 135),
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
function useArmScene(containerRef, joints, wireframe, onJointDelta, onIkDrag, sceneApiRef) {
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

    // ระยะกล้องอ้างอิง (= ค่าเริ่มต้นของ controls.radius) ใช้คำนวณสเกล gizmo ให้ขนาดบนจอคงที่
    const GIZMO_REF_DISTANCE = controls.radius;

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
    const handleDrag = {
      active: false,
      mode: null,
      spec: null,
      lastX: 0,
      lastY: 0,
      plane: new THREE.Plane(),
      lineOrigin: new THREE.Vector3(),
      lineDir: new THREE.Vector3(),
      lastPoint: new THREE.Vector3(),
      lastT: 0,
    };
    const AXIS_VECTORS = {
      x: new THREE.Vector3(1, 0, 0),
      y: new THREE.Vector3(0, 1, 0),
      z: new THREE.Vector3(0, 0, 1),
    };
    const _p0 = new THREE.Vector3();
    const _p1 = new THREE.Vector3();
    // scratch vectors used while dragging the IK gizmo (must not be reallocated per frame)
    const _dragLineDir = new THREE.Vector3();
    const _camForward = new THREE.Vector3();
    const _camRight = new THREE.Vector3();
    const _planeNormal = new THREE.Vector3();
    const _intersectPoint = new THREE.Vector3();

    // ---- Hover feedback state (ทำให้เห็นชัดว่ากำลังจะจับแกนไหนก่อนลาก) ----
    let hoveredKey = null; // "x" | "y" | "z" | "free" | null
    function setHoverVisual(key) {
      if (key === hoveredKey) return;
      const s = sceneRef.current;
      // คืนค่าตัวที่ hover ค้างไว้ก่อนหน้ากลับเป็นปกติ
      if (hoveredKey && s?.gizmoParts?.[hoveredKey]) {
        const prev = s.gizmoParts[hoveredKey];
        prev.grp.scale.setScalar(1);
        prev.mats.forEach((m, i) => m.color.copy(prev.baseColors[i]));
      }
      hoveredKey = key;
      if (key && s?.gizmoParts?.[key]) {
        const cur = s.gizmoParts[key];
        cur.grp.scale.setScalar(1.35);
        cur.mats.forEach((m) => m.color.set(0xffffff));
      }
      renderer.domElement.style.cursor = key ? "grab" : "grab";
    }

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

    // แปลงตำแหน่ง pointer event -> THREE.Ray จากกล้อง (ใช้กับ ray.intersectPlane ตอนลาก gizmo)
    function pointerRay(e) {
      const rect = renderer.domElement.getBoundingClientRect();
      pointerNDC.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      pointerNDC.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointerNDC, camera);
      return raycaster.ray;
    }

    function onPointerDown(e) {
      const spec = pickHandle(e);
      const s = sceneRef.current;
      if (spec && spec.axis && onIkDragRef.current && s && s.translateGizmo) {
        // ลากลูกศร -> โหมด IK translate ตามแกนเดียว (คำนวณผ่าน ray-plane intersection จริง)
        const gizmoPos = s.translateGizmo.position;
        _dragLineDir.copy(AXIS_VECTORS[spec.axis]).normalize();
        camera.getWorldDirection(_camForward);
        _camRight.crossVectors(_camForward, _dragLineDir);
        if (_camRight.lengthSq() < 1e-6) _camRight.crossVectors(_camForward, camera.up);
        _planeNormal.crossVectors(_dragLineDir, _camRight).normalize();
        handleDrag.plane.setFromNormalAndCoplanarPoint(_planeNormal, gizmoPos);
        handleDrag.lineOrigin.copy(gizmoPos);
        handleDrag.lineDir.copy(_dragLineDir);
        const ray = pointerRay(e);
        handleDrag.lastT = ray.intersectPlane(handleDrag.plane, _intersectPoint)
          ? _intersectPoint.clone().sub(gizmoPos).dot(_dragLineDir)
          : 0;
        handleDrag.active = true;
        handleDrag.mode = "ik-axis";
        handleDrag.spec = spec;
        renderer.domElement.style.cursor = "grabbing";
        return;
      }
      if (spec && spec.free && onIkDragRef.current && s && s.translateGizmo) {
        // ลากลูกบอลกลาง -> โหมด IK translate อิสระ 3 แกนพร้อมกัน (ลากปลายมือคีบไปไหนก็ได้)
        const gizmoPos = s.translateGizmo.position;
        camera.getWorldDirection(_camForward);
        handleDrag.plane.setFromNormalAndCoplanarPoint(_camForward, gizmoPos);
        const ray = pointerRay(e);
        handleDrag.lastPoint.copy(
          ray.intersectPlane(handleDrag.plane, _intersectPoint) ? _intersectPoint : gizmoPos
        );
        handleDrag.active = true;
        handleDrag.mode = "ik-free";
        handleDrag.spec = spec;
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
      if (handleDrag.active && handleDrag.mode === "ik-axis") {
        const s = sceneRef.current;
        const modelScale = s?.modelScale || 1;
        const ray = pointerRay(e);
        if (ray.intersectPlane(handleDrag.plane, _intersectPoint)) {
          const t = _intersectPoint.clone().sub(handleDrag.lineOrigin).dot(handleDrag.lineDir);
          const deltaT = t - handleDrag.lastT;
          handleDrag.lastT = t;
          onIkDragRef.current?.(handleDrag.spec.axis, deltaT / modelScale);
        }
        return;
      }
      if (handleDrag.active && handleDrag.mode === "ik-free") {
        const s = sceneRef.current;
        const modelScale = s?.modelScale || 1;
        const ray = pointerRay(e);
        if (ray.intersectPlane(handleDrag.plane, _intersectPoint)) {
          const delta = _intersectPoint.clone().sub(handleDrag.lastPoint);
          handleDrag.lastPoint.copy(_intersectPoint);
          onIkDragRef.current?.("xyz", {
            x: delta.x / modelScale,
            y: delta.y / modelScale,
            z: delta.z / modelScale,
          });
        }
        return;
      }
      if (!controls.dragging) {
        // ไม่ได้ลากอะไรอยู่ -> เช็คว่าเมาส์ชี้ตรงแกน/ลูกบอลกลางไหม เพื่อไฮไลต์ก่อนกดจับ
        const spec = pickHandle(e);
        setHoverVisual(spec ? (spec.axis || (spec.free ? "free" : null)) : null);
        return;
      }
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
    function onPointerLeave() {
      if (!handleDrag.active) setHoverVisual(null);
    }
    renderer.domElement.addEventListener("pointerdown", onPointerDown);
    window.addEventListener("pointermove", onPointerMove);
    window.addEventListener("pointerup", onPointerUp);
    renderer.domElement.addEventListener("pointerleave", onPointerLeave);
    renderer.domElement.addEventListener("wheel", onWheel, { passive: false });

    const _eeWorldPos = new THREE.Vector3();
    const _tipL = new THREE.Vector3();
    const _tipR = new THREE.Vector3();
    let raf;
    function tick() {
      applyCamera();
      const s = sceneRef.current;
      if (s && s.ready && s.fingerLTip && s.fingerRTip && s.translateGizmo) {
        // ตำแหน่ง J5 จริง = จุดกึ่งกลางระหว่างปลายนิ้วซ้าย-ขวา (world space) ทุกเฟรม
        s.fingerLTip.getWorldPosition(_tipL);
        s.fingerRTip.getWorldPosition(_tipR);
        _eeWorldPos.copy(_tipL).add(_tipR).multiplyScalar(0.5);
        // สำคัญ: sync ตำแหน่ง gizmo กับปลายนิ้วจริง "เฉพาะตอนไม่ได้ลากอยู่" เท่านั้น
        // เดิม sync ทุกเฟรมไม่มีเงื่อนไข ทำให้ตอนลาก ถ้าแขนตามไม่ทันแม้แค่เฟรมเดียว (เช่น
        // ไปเกือบสุดพิสัย/CCD ยังลู่เข้าไม่ครบ) ลูกศรจะถูกดึงกลับไปที่ตำแหน่งแขนจริงทันที
        // ทำให้รู้สึกว่า "ลากแล้วไม่ขยับตาม" ทั้งที่จริงๆ ขยับแค่ตามไม่ทันชั่วคราว
        // ตอนนี้ระหว่างลาก ตำแหน่ง gizmo จะถูกตั้งค่าตรงๆ จาก handleIkDrag (เท่ากับจุดที่ลาก
        // ไปจริง 1:1 ตามเมาส์เสมอ) แล้วค่อย snap กลับไปตำแหน่งจริงตอนปล่อยเมาส์
        if (!handleDrag.active) {
          s.translateGizmo.position.copy(_eeWorldPos);
        }
        // คงขนาด gizmo ให้ดู "เท่าเดิมบนจอ" ไม่ว่าจะซูมเข้า/ออกแค่ไหน
        // (ไม่งั้นตอนซูมออกลูกศรจะเล็กจิ๋วจนลากยากมาก) — สเกลตามระยะกล้องจริง
        const camDist = camera.position.distanceTo(s.translateGizmo.position);
        const gizmoScale = camDist / GIZMO_REF_DISTANCE;
        s.translateGizmo.scale.setScalar(gizmoScale);
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
      fingerLTip: null,
      fingerRTip: null,
      modelScale: 5,
      allMeshes: [],
      pickables: [],
      gizmoParts: null,
      translateGizmo: null,
      rotateGizmo: null,
      ready: false,
    };
    if (sceneApiRef) sceneApiRef.current = sceneRef.current;

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

      // ---- จุดปลายนิ้วจริง (J5) — ไม่เดาระยะ (L4) อีกต่อไป ----
      // หาจุด "ปลายนิ้ว" จริงจาก bounding box ของ geometry นิ้วซ้าย/ขวาแต่ละอัน
      // (มุมที่ไกลสุดตามแกนยื่นออกของนิ้ว = ปลายนิ้ว) แล้วผูกเป็น marker
      // ลูกของนิ้วนั้น ๆ เพื่อให้ตำแหน่งโลกอัปเดตถูกต้องตามข้อต่อ/การหุบ-กางนิ้วเสมอ
      function attachFingerTipMarker(fingerMesh, name) {
        if (!fingerMesh.geometry.boundingBox) fingerMesh.geometry.computeBoundingBox();
        const bb = fingerMesh.geometry.boundingBox;
        const tipLocal = new THREE.Vector3(
          bb.max.x,
          (bb.min.y + bb.max.y) / 2,
          (bb.min.z + bb.max.z) / 2
        );
        const marker = new THREE.Object3D();
        marker.name = name;
        marker.position.copy(tipLocal);
        fingerMesh.add(marker);
        return marker;
      }
      s.fingerLTip = attachFingerTipMarker(s.fingerL, "FingerL_Tip");
      s.fingerRTip = attachFingerTipMarker(s.fingerR, "FingerR_Tip");

      // ---- คาลิเบรต L4 จากโมเดลจริง (ไม่เดาอีกต่อไป) ----
      // บังคับข้อต่อเข้าสู่ pose HOME ก่อน (มือคีบอยู่ในแนวระดับตามสมมติฐานของสมการ IK ที่ J4
      // ชดเชยให้มือคีบชี้แนวนอนเสมอ) แล้ววัดระยะจริงจากจุดหมุนข้อมือ (J4) ถึงจุดกึ่งกลางปลายนิ้ว
      // จริง หารด้วย scale ของโมเดล (5x) เพื่อแปลงกลับเป็นหน่วยเมตรแบบเดียวกับ L1/L2/L3
      // — ความคลาดเคลื่อนของค่าคงที่นี้คือสาเหตุหลักที่ลากแกนหนึ่งแล้วปลายมือคีบเบี้ยวไปแกนอื่นด้วย
      {
        const d = THREE.MathUtils.degToRad;
        s.baseGroup.rotation.y = d(HOME.j1);
        s.shoulder.rotation.x = d(-HOME.j2);
        s.elbow.rotation.y = d(-HOME.j3);
        s.wrist.rotation.y = d(-HOME.j4);
        s.baseGroup.updateMatrixWorld(true);

        const wristWorld = new THREE.Vector3();
        const tipLWorld = new THREE.Vector3();
        const tipRWorld = new THREE.Vector3();
        s.wrist.getWorldPosition(wristWorld);
        s.fingerLTip.getWorldPosition(tipLWorld);
        s.fingerRTip.getWorldPosition(tipRWorld);
        const tipMidWorld = tipLWorld.add(tipRWorld).multiplyScalar(0.5);

        const modelScale = model.scale.x || 1;
        const measuredL4 = wristWorld.distanceTo(tipMidWorld) / modelScale;
        if (measuredL4 > 0) L4 = measuredL4;
      }

      // ---- Numeric IK (CCD) จากตำแหน่งปลายนิ้วจริง ----
      // สูตร solveIK() เดิมเป็นแบบ analytic 2-link ที่ "สมมติ" ว่าระยะ L4 ยื่นออกไปตาม
      // แนวรัศมี (แนวเดียวกับ r) เสมอ — แต่จากตัว model จริง จุดต่อของ FingerBase กับ
      // ArmGriper ยื่นไปตามแกน Y ของข้อมือ ไม่ใช่แกน X ทำให้สมมติฐานนั้นผิดในหลาย pose
      // (ยิ่ง J2/J3/J4 เบี่ยงจากตำแหน่ง HOME มาก ยิ่งเพี้ยนมาก) — เป็นสาเหตุที่ลากลูกศร
      // แล้วปลายมือคีบไปไม่ถึงจุดที่ลาก โดยเฉพาะตำแหน่งที่ไกลจาก HOME
      //
      // ฟังก์ชันนี้แก้ปัญหาตรงจุด: ไล่ปรับมุมข้อต่อ (CCD) โดยอ้างอิงตำแหน่ง "ปลายนิ้วจริง"
      // (จุดกึ่งกลาง fingerLTip/fingerRTip) จากโครงกระดูกจริงของโมเดลทุกครั้ง จึงแม่นยำ
      // ไม่ว่ารูปทรง/ทิศทางของมือคีบจะเป็นอย่างไรก็ตาม ไม่ต้องพึ่งสมมติฐานเรื่องทิศทาง L4 เลย
      const ikChain = [
        { obj: s.baseGroup, axis: "y", limMin: -180, limMax: 180 },
        { obj: s.shoulder, axis: "x", limMin: -90, limMax: 90 },
        { obj: s.elbow, axis: "y", limMin: -135, limMax: 135 },
        { obj: s.wrist, axis: "y", limMin: -135, limMax: 135 },
      ];
      const _ccdP = new THREE.Vector3();
      const _ccdE = new THREE.Vector3();
      const _ccdV1 = new THREE.Vector3();
      const _ccdV2 = new THREE.Vector3();
      const _ccdCross = new THREE.Vector3();
      const _ccdQ = new THREE.Quaternion();
      const _ccdAxisLocalX = new THREE.Vector3(1, 0, 0);
      const _ccdAxisLocalY = new THREE.Vector3(0, 1, 0);
      const _ccdTipL = new THREE.Vector3();
      const _ccdTipR = new THREE.Vector3();

      function getTipMidWorld(out) {
        s.fingerLTip.getWorldPosition(_ccdTipL);
        s.fingerRTip.getWorldPosition(_ccdTipR);
        return out.copy(_ccdTipL).add(_ccdTipR).multiplyScalar(0.5);
      }

      // ปรับมุมข้อต่อ (mutate live scene ตรงๆ) ให้ปลายนิ้วจริงเข้าใกล้ targetWorld (สเกลเดียวกับฉาก x5)
      // มากที่สุดภายใต้ขอบเขตมุมของแต่ละข้อ แล้วคืนค่ามุม j1-j4 (องศา) ที่ได้
      function numericIK(targetWorld, iterations = 14) {
        for (let iter = 0; iter < iterations; iter++) {
          for (let i = ikChain.length - 1; i >= 0; i--) {
            const link = ikChain[i];
            const joint = link.obj;
            joint.getWorldPosition(_ccdP);
            getTipMidWorld(_ccdE);
            joint.getWorldQuaternion(_ccdQ);
            const axisLocal = link.axis === "x" ? _ccdAxisLocalX : _ccdAxisLocalY;
            const axisWorld = axisLocal.clone().applyQuaternion(_ccdQ).normalize();

            _ccdV1.copy(_ccdE).sub(_ccdP);
            _ccdV2.copy(targetWorld).sub(_ccdP);
            _ccdV1.addScaledVector(axisWorld, -_ccdV1.dot(axisWorld));
            _ccdV2.addScaledVector(axisWorld, -_ccdV2.dot(axisWorld));
            if (_ccdV1.lengthSq() < 1e-10 || _ccdV2.lengthSq() < 1e-10) continue;
            _ccdV1.normalize();
            _ccdV2.normalize();
            _ccdCross.crossVectors(_ccdV1, _ccdV2);
            const sinA = _ccdCross.dot(axisWorld);
            const cosA = _ccdV1.dot(_ccdV2);
            let angle = Math.atan2(sinA, cosA) * 0.75; // damping กันสั่น/แกว่ง

            const limRad = THREE.MathUtils.degToRad;
            const rot = joint.rotation;
            const cur = link.axis === "x" ? rot.x : rot.y;
            let next = cur + angle;
            next = Math.max(limRad(link.limMin), Math.min(limRad(link.limMax), next));
            if (link.axis === "x") rot.x = next; else rot.y = next;
            joint.updateMatrixWorld(true);
          }
        }
        getTipMidWorld(_ccdE);
        const reachedDist = _ccdE.distanceTo(targetWorld);
        return {
          j1: THREE.MathUtils.radToDeg(s.baseGroup.rotation.y),
          j2: -THREE.MathUtils.radToDeg(s.shoulder.rotation.x),
          j3: -THREE.MathUtils.radToDeg(s.elbow.rotation.y),
          j4: -THREE.MathUtils.radToDeg(s.wrist.rotation.y),
          reachedDist, // ระยะที่เหลือจากเป้าหมายจริง (สเกล x5) — ใช้เช็ค unreachable
        };
      }

      // ตั้งข้อต่อทั้งหมดไปที่ค่า degrees ที่กำหนด (ใช้ seed ก่อนรัน CCD จากมุมที่รู้แน่ชัด เช่น HOME)
      function setChainDegrees(j) {
        const d = THREE.MathUtils.degToRad;
        s.baseGroup.rotation.y = d(j.j1);
        s.shoulder.rotation.x = d(-j.j2);
        s.elbow.rotation.y = d(-j.j3);
        s.wrist.rotation.y = d(-j.j4);
        s.baseGroup.updateMatrixWorld(true);
      }

      s.numericIK = numericIK;
      s.setChainDegrees = setChainDegrees;
      s.getTipMidWorld = (out) => getTipMidWorld(out || new THREE.Vector3());

      // ---- Gizmo ที่ปลายมือคีบ (J5) ----
      // ลูกศรเลื่อน XYZ เท่านั้น (world-aligned, ไม่หมุนตามข้อต่อ)
      // อยู่ที่ "ปลายมือคีบจริง" (จุดกึ่งกลางระหว่างนิ้วซ้าย-ขวา คือตำแหน่ง J5)
      // ลากลูกศรแกนไหน -> ปลายมือคีบขยับไปทางแกนนั้นในพิกัดโลกจริง แล้วคำนวณ IK
      // ย้อนกลับไปหามุมของทุกข้อต่อ (J1-J4) ให้ปลายมือคีบไปถึงตำแหน่งนั้น (แดง=X เขียว=Y น้ำเงิน=Z)
      //
      // หมายเหตุ: gizmo ถูก add เข้า scene ตรงๆ (ไม่ใช่ลูกของโมเดล) แต่ตำแหน่งที่อ้างอิง
      // (ปลายนิ้วจริง) อยู่ใน "พิกัดโลกที่ผ่าน model.scale(5x) แล้ว" อยู่แล้ว จึงไม่ต้องคูณ
      // modelScale ซ้ำอีกชั้นตอนกำหนดขนาด — เดิมคูณซ้ำ (0.2 * 5 = 1.0) ทำให้ลูกศรใหญ่เท่าๆ
      // กับความยาวแขนทั้งชุด ลากยากเพราะแกนทับกันเอง จึงตัดตัวคูณซ้ำนี้ออกและลดขนาดฐานลง
      const gizmoLen = 0.17;

      // พื้นที่คลิก "โปร่งใส" ใหญ่กว่ารูปที่มองเห็นจริงพอประมาณ เพื่อให้จิ้ม/ลากง่ายขึ้น
      const HIT_PADDING = 2.0;


      function makeArrow(dir, color, axis) {
        const shaftLen = gizmoLen * 0.7;
        const headLen = gizmoLen * 0.3;
        const shaftRadius = gizmoLen * 0.03;
        const headRadius = gizmoLen * 0.09;
        const mat = new THREE.MeshBasicMaterial({ color, depthTest: false });
        const shaft = new THREE.Mesh(new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLen, 8), mat);
        shaft.position.y = shaftLen / 2;
        const head = new THREE.Mesh(new THREE.ConeGeometry(headRadius, headLen, 10), mat);
        head.position.y = shaftLen + headLen / 2;

        // hit-box โปร่งใสครอบทั้งแกน (หนากว่าของจริงมาก) ใช้แค่สำหรับ raycast ไม่ได้ render ให้เห็น
        const hitMat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0, depthTest: false });
        const hitBox = new THREE.Mesh(
          new THREE.CylinderGeometry(shaftRadius * HIT_PADDING, headRadius * HIT_PADDING, gizmoLen, 10),
          hitMat
        );
        hitBox.position.y = gizmoLen / 2;
        hitBox.renderOrder = 998;

        const grp = new THREE.Group();
        grp.add(shaft, head, hitBox);
        grp.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
        grp.renderOrder = 999;
        const userData = { axis };
        shaft.userData = userData;
        head.userData = userData;
        hitBox.userData = userData;
        return { grp, pickables: [shaft, head, hitBox], mats: [mat], baseColors: [new THREE.Color(color)] };
      }

      s.pickables = [];
      s.gizmoParts = {};

      // ---- ลูกศรเลื่อน XYZ (world space) -> ผูกกับ IK จริง ----
      const translateGizmo = new THREE.Group();
      translateGizmo.name = "IK_TranslateGizmo";
      const arrows = [
        { key: "x", data: makeArrow(new THREE.Vector3(1, 0, 0), 0xef4444, "x") },
        { key: "y", data: makeArrow(new THREE.Vector3(0, 1, 0), 0x22c55e, "y") },
        { key: "z", data: makeArrow(new THREE.Vector3(0, 0, 1), 0x3b6cf6, "z") },
      ];
      arrows.forEach(({ key, data }) => {
        translateGizmo.add(data.grp);
        s.pickables.push(...data.pickables);
        s.gizmoParts[key] = { grp: data.grp, mats: data.mats, baseColors: data.baseColors };
      });

      // ---- ลูกบอลกลาง -> ลากอิสระ 3 แกนพร้อมกัน (ง่ายกว่าเล็งแกนเดี่ยวสำหรับผู้เริ่มต้น) ----
      const freeRadius = gizmoLen * 0.16;
      const freeMat = new THREE.MeshBasicMaterial({
        color: 0xe7ebf5,
        transparent: true,
        opacity: 0.85,
        depthTest: false,
      });
      const freeBall = new THREE.Mesh(new THREE.SphereGeometry(freeRadius, 20, 16), freeMat);
      freeBall.renderOrder = 999;
      const freeHitMat = new THREE.MeshBasicMaterial({ color: 0xe7ebf5, transparent: true, opacity: 0, depthTest: false });
      const freeHit = new THREE.Mesh(new THREE.SphereGeometry(freeRadius * 2.2, 16, 12), freeHitMat);
      freeHit.renderOrder = 998;
      const freeGrp = new THREE.Group();
      freeGrp.add(freeBall, freeHit);
      const freeUserData = { free: true };
      freeBall.userData = freeUserData;
      freeHit.userData = freeUserData;
      translateGizmo.add(freeGrp);
      s.pickables.push(freeBall, freeHit);
      s.gizmoParts.free = { grp: freeGrp, mats: [freeMat], baseColors: [new THREE.Color(0xe7ebf5)] };

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
      renderer.domElement.removeEventListener("pointerleave", onPointerLeave);
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

  // ---- Pose Library: บันทึกตำแหน่งแขนกล (มุมข้อต่อ J1-J5) เก็บเป็นท่าไว้เรียกใช้ภายหลัง ----
  const POSES_STORAGE_KEY = "anr_robot_saved_poses_v1";
  const [savedPoses, setSavedPoses] = useState(() => {
    try {
      const raw = localStorage.getItem(POSES_STORAGE_KEY);
      return raw ? JSON.parse(raw) : [];
    } catch {
      return [];
    }
  });
  const [poseNameDraft, setPoseNameDraft] = useState("");

  useEffect(() => {
    try {
      localStorage.setItem(POSES_STORAGE_KEY, JSON.stringify(savedPoses));
    } catch {
      /* เก็บไม่ได้ (เช่น storage เต็ม) — ปล่อยผ่าน ไม่ต้องล้มทั้งแอป */
    }
  }, [savedPoses]);

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
    const s = sceneApiRef.current;
    if (!s || !s.ready) return;
    const tx = parseFloat(x) || 0, ty = parseFloat(y) || 0, tz = parseFloat(z) || 0;

    // เช็ค reachability แบบหยาบก่อนด้วยสูตร analytic (เร็ว, กันเคสไกลเกินพิสัยชัดเจน)
    const rough = solveIK(tx, ty, tz, jointsRef.current);
    if (!rough.ok) {
      setIkError("ตำแหน่งอยู่นอกพิสัยของแขนกล (Unreachable)");
      return;
    }

    // seed จากผลลัพธ์ analytic (ซึ่งเลือก configuration ข้อศอกที่ "เป็นธรรมชาติ" ไว้แล้ว
    // จากการเช็ค A/B ในขอบเขตข้อต่อ) แล้วใช้ CCD แค่ไม่กี่รอบเพื่อแก้ค่าคลาดเคลื่อนเล็กน้อย
    // จากสมมติฐานทิศทาง L4 เท่านั้น — เดิม seed จาก HOME แล้วปล่อย CCD ไล่เองอิสระ ทำให้
    // บางครั้งข้อศอก/ไหล่ไปเจอ configuration ที่ผิดธรรมชาติ (งอย้อนกลับ/บิดผิดท่า)
    const modelScale = s.modelScale || 1;
    s.setChainDegrees({ j1: rough.j1, j2: rough.j2, j3: rough.j3, j4: rough.j4 });
    const targetWorld = new THREE.Vector3(tx, ty, tz).multiplyScalar(modelScale);
    const result = s.numericIK(targetWorld, 6);
    if (result.reachedDist > 0.015 * modelScale) {
      setIkError("ตำแหน่งอยู่นอกพิสัยของแขนกล (Unreachable)");
      return;
    }
    setIkError("");
    const newTargets = { j1: result.j1, j2: result.j2, j3: result.j3, j4: result.j4, j5: jointsRef.current.j5 };
    setTargets(newTargets);

    // เริ่มจากองศาเริ่มต้น (HOME) เสมอ ก่อนเคลื่อนไปยังเป้าหมายที่คำนวณจาก IK
    runFromHome(newTargets, motionType, () => {
      ikTargetRef.current = { x: tx, y: ty, z: tz };
    });
  }, [moving, motionType, ikTarget, runFromHome]);

  // ---- ลากลูกศร XYZ ที่ปลายแขน -> ขยับตำแหน่งจริงแบบเรียลไทม์แล้วคำนวณ IK ย้อนกลับ ----
  // ทุกข้อต่อ (J1-J4) จะถูกคำนวณใหม่ให้ปลายแขนไปอยู่ที่ตำแหน่ง XYZ เป้าหมายเสมอ
  // ใช้ numeric CCD จากตำแหน่งปลายนิ้วจริง (ไม่ใช่สูตร analytic) จาก "ท่าปัจจุบัน" ของแขน
  // (ไม่ต้อง reset ไป HOME ก่อน) เพื่อให้ลากได้ต่อเนื่องลื่นไหลแบบเรียลไทม์
  const handleIkDrag = useCallback((axis, delta) => {
    if (moving) return;
    const s = sceneApiRef.current;
    if (!s || !s.ready) return;
    const cur = ikTargetRef.current;

    // กันการกระโดดผิดปกติในเฟรมเดียว (เช่น มุมกล้องเกือบขนานกับระนาบลาก ทำให้ ray-plane
    // intersection ขยับไกลผิดปกติจากการขยับเมาส์เพียงเล็กน้อย) — จำกัด step ต่อเฟรมไว้
    const MAX_STEP = 0.05; // เมตร ต่อ pointermove event
    const clampStep = (v) => Math.max(-MAX_STEP, Math.min(MAX_STEP, v));

    const next =
      axis === "xyz"
        ? {
            ...cur,
            x: (parseFloat(cur.x) || 0) + clampStep(delta.x),
            y: (parseFloat(cur.y) || 0) + clampStep(delta.y),
            z: (parseFloat(cur.z) || 0) + clampStep(delta.z),
          }
        : { ...cur, [axis]: (parseFloat(cur[axis]) || 0) + clampStep(delta) };

    const modelScale = s.modelScale || 1;

    // seed จากผลลัพธ์ analytic (เลือก configuration ข้อศอกที่เป็นธรรมชาติให้แล้ว) ทุกครั้งที่ลาก
    // แทนที่จะปล่อยให้ CCD ไล่ต่อจาก "ท่าปัจจุบัน" อิสระไปเรื่อยๆ ซึ่งสะสมความเบี้ยวจนข้อศอก/ไหล่
    // ไปอยู่ใน configuration ที่ผิดธรรมชาติได้เมื่อลากต่อเนื่องนานๆ — CCD ทำหน้าที่แค่ "แก้ค่า
    // คลาดเคลื่อนเล็กน้อย" จากสมมติฐานทิศทาง L4 ของสูตร analytic เท่านั้น ไม่ใช่หาท่าทางเอง
    const rough = solveIK(
      parseFloat(next.x) || 0,
      parseFloat(next.y) || 0,
      parseFloat(next.z) || 0,
      jointsRef.current
    );
    if (rough.ok) {
      s.setChainDegrees({ j1: rough.j1, j2: rough.j2, j3: rough.j3, j4: rough.j4 });
    }
    const targetWorld = new THREE.Vector3(
      parseFloat(next.x) || 0,
      parseFloat(next.y) || 0,
      parseFloat(next.z) || 0
    ).multiplyScalar(modelScale);
    // CCD แค่ไม่กี่รอบพอ เพราะ seed จาก analytic ใกล้คำตอบอยู่แล้ว ไม่ต้องไล่ไกล
    const result = s.numericIK(targetWorld, rough.ok ? 5 : 14);

    // ซิงก์ ikTarget กับ "ตำแหน่งที่ไปถึงจริง" เสมอ (ไม่ใช่ตำแหน่งที่ลากขอไว้ดิบๆ)
    // เดิมถ้าไปไม่ถึงจะ setChainDegrees สะบัดกลับไปตำแหน่งก่อนหน้าทันที แล้วพอลากกลับเข้า
    // พิสัยอีกครั้งค่าที่สะสมไว้ทำให้กระโดดพรวดเดียว — อาการนี้คือ "กระตุก" ที่เจอ
    // ตอนนี้ให้แขนหยุดค้างที่ "ขอบเขตที่ไปถึงได้จริง" เสมอ ไม่มีการสะบัดกลับ/กระโดดทีหลัง
    const actualTip = s.getTipMidWorld(new THREE.Vector3()).divideScalar(modelScale);
    const actualNext = { x: actualTip.x, y: actualTip.y, z: actualTip.z };

    ikTargetRef.current = actualNext;
    setIkTarget(actualNext);
    setIkError(
      result.reachedDist > 0.02 * modelScale
        ? "ตำแหน่งอยู่นอกพิสัยของแขนกล — แขนค้างที่ขอบเขตที่ไปถึงได้จริง"
        : ""
    );
    const newJoints = { j1: result.j1, j2: result.j2, j3: result.j3, j4: result.j4, j5: jointsRef.current.j5 };
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
  const sceneApiRef = useRef(null);
  const { modelReady, modelError } = useArmScene(viewerRef, joints, false, handleJointDelta, handleIkDrag, sceneApiRef);

  // เมื่อโมเดลโหลดเสร็จและ L4 ถูกคาลิเบรตจากตำแหน่งจริงแล้ว รีเฟรชค่า IK readout/target
  // ให้ตรงกับค่าที่คาลิเบรตใหม่ (ก่อนหน้านี้ตอน mount ครั้งแรกยังใช้ค่า L4 fallback อยู่)
  useEffect(() => {
    if (!modelReady) return;
    const fk = forwardKinematics(jointsRef.current.j1, jointsRef.current.j2, jointsRef.current.j3);
    ikTargetRef.current = fk;
    setIkTarget(fk);
  }, [modelReady]);

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

  // ---- ปุ่ม Home: เคลื่อนแขนกลกลับตำแหน่งเริ่มต้น (HOME) ทันที ----
  const handleHome = useCallback(() => {
    if (moving || !jointsRef.current) return;
    setMoving(true);
    const fromNow = { ...jointsRef.current };
    animateJoints({
      from: fromNow,
      to: HOME,
      duration: HOME_RESET_MS,
      motion: "PTP",
      onDone: () => {
        setMoving(false);
        setTargets(HOME);
        const fk = forwardKinematics(HOME.j1, HOME.j2, HOME.j3);
        ikTargetRef.current = fk;
        setIkTarget(fk);
      },
    });
  }, [moving, animateJoints]);

  // ---- บันทึกท่าปัจจุบันของแขนกล (มุมข้อต่อ J1-J5) เป็นท่าใหม่ในไลบรารี ----
  const handleSavePose = useCallback(() => {
    const name = poseNameDraft.trim() || `ท่า ${savedPoses.length + 1}`;
    const pose = {
      id: `${Date.now()}_${Math.random().toString(36).slice(2, 7)}`,
      name,
      joints: { ...jointsRef.current },
      createdAt: Date.now(),
    };
    setSavedPoses((prev) => [...prev, pose]);
    setPoseNameDraft("");
  }, [poseNameDraft, savedPoses.length]);

  // ---- เคลื่อนแขนกลไปยังท่าที่บันทึกไว้ (ผ่าน HOME ก่อนเสมอ ตามรูปแบบ Move ปกติ) ----
  const handleGoToPose = useCallback((pose) => {
    if (moving) return;
    setTargets(pose.joints);
    runFromHome(pose.joints, motionType, () => {
      const fk = forwardKinematics(pose.joints.j1, pose.joints.j2, pose.joints.j3);
      ikTargetRef.current = fk;
      setIkTarget(fk);
    });
  }, [moving, motionType, runFromHome]);

  const handleDeletePose = useCallback((id) => {
    setSavedPoses((prev) => prev.filter((p) => p.id !== id));
  }, []);

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
          <button
            onClick={handleHome}
            disabled={moving || !modelReady}
            title="กลับตำแหน่งเริ่มต้น (Home)"
            className="flex items-center gap-1.5 px-3.5 py-1.5 rounded-full text-xs font-medium transition-colors"
            style={{
              background: C.panelAlt,
              color: moving || !modelReady ? C.subDim : C.text,
              border: `1px solid ${C.borderSoft}`,
              cursor: moving || !modelReady ? "default" : "pointer",
              opacity: moving || !modelReady ? 0.6 : 1,
            }}
          >
            <HomeIcon size={13} />
            Home
          </button>

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

          {/* ---------------- Pose Library panel (บันทึก/เรียกใช้ท่า) ---------------- */}
          <div
            className="absolute top-4 left-4 w-64 rounded-2xl overflow-hidden shadow-2xl flex flex-col"
            style={{
              maxHeight: "calc(100% - 32px)",
              background: "rgba(16,21,42,0.92)",
              border: `1px solid ${C.border}`,
              backdropFilter: "blur(6px)",
            }}
          >
            <div className="flex items-center gap-2 px-4 pt-3.5 pb-2.5 shrink-0" style={{ borderBottom: `1px solid ${C.borderSoft}` }}>
              <BookMarked size={14} color={C.accent} />
              <span className="text-xs font-semibold" style={{ color: C.text }}>Pose Library</span>
            </div>

            {/* บันทึกท่าปัจจุบัน */}
            <div className="px-4 pt-3 pb-3 flex gap-2 shrink-0" style={{ borderBottom: `1px solid ${C.borderSoft}` }}>
              <input
                type="text"
                value={poseNameDraft}
                onChange={(e) => setPoseNameDraft(e.target.value)}
                onKeyDown={(e) => { if (e.key === "Enter" && !moving && modelReady) handleSavePose(); }}
                placeholder={`ท่า ${savedPoses.length + 1}`}
                disabled={moving || !modelReady}
                className="flex-1 min-w-0 rounded-lg px-2.5 py-1.5 text-[12px] outline-none"
                style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text, opacity: moving ? 0.5 : 1 }}
              />
              <button
                onClick={handleSavePose}
                disabled={moving || !modelReady}
                title="บันทึกท่าปัจจุบัน"
                className="flex items-center justify-center rounded-lg px-2.5 shrink-0 transition-colors"
                style={{
                  background: C.accent,
                  color: "#fff",
                  cursor: moving || !modelReady ? "default" : "pointer",
                  opacity: moving || !modelReady ? 0.5 : 1,
                }}
              >
                <Save size={14} />
              </button>
            </div>

            {/* รายการท่าที่บันทึกไว้ */}
            <div className="overflow-y-auto px-2.5 py-2 flex flex-col gap-1.5" style={{ minHeight: 0 }}>
              {savedPoses.length === 0 && (
                <div className="px-1.5 py-3 text-[11px] text-center" style={{ color: C.subDim }}>
                  ยังไม่มีท่าที่บันทึกไว้ — ตั้งชื่อแล้วกดปุ่ม Save ด้านบน
                </div>
              )}
              {savedPoses.map((pose) => (
                <div
                  key={pose.id}
                  className="flex items-center gap-1.5 rounded-lg px-2.5 py-2"
                  style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}
                >
                  <div className="flex-1 min-w-0">
                    <div className="text-[12px] font-medium truncate" style={{ color: C.text }}>{pose.name}</div>
                    <div className="text-[10px] font-mono truncate" style={{ color: C.subDim }}>
                      J1 {pose.joints.j1.toFixed(0)}° · J2 {pose.joints.j2.toFixed(0)}° · J3 {pose.joints.j3.toFixed(0)}°
                    </div>
                  </div>
                  <button
                    onClick={() => handleGoToPose(pose)}
                    disabled={moving || !modelReady}
                    title="เคลื่อนไปยังท่านี้"
                    className="flex items-center justify-center rounded-md p-1.5 shrink-0 transition-colors"
                    style={{ color: moving || !modelReady ? C.subDim : C.accent, cursor: moving || !modelReady ? "default" : "pointer" }}
                  >
                    <PlayCircle size={16} />
                  </button>
                  <button
                    onClick={() => handleDeletePose(pose.id)}
                    disabled={moving}
                    title="ลบท่านี้"
                    className="flex items-center justify-center rounded-md p-1.5 shrink-0 transition-colors"
                    style={{ color: moving ? C.subDim : C.red, cursor: moving ? "default" : "pointer" }}
                  >
                    <Trash2 size={14} />
                  </button>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
