import React, { useEffect, useRef, useState, useCallback } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import robotModel from "./assets/robotarm.glb";
import {
  Bot,
  Gamepad2,
  PlayCircle,
  Activity,
  Info,
  Move,
  Crosshair,
  ZoomIn,
  ZoomOut,
  Box as BoxIcon,
  Save,
  RotateCcw,
  Square,
  Send,
  Minus,
  Plus,
  Clock,
  CalendarDays,
  ChevronDown,
  Wifi,
  Trash2,
  ArrowUp,
  ArrowDown,
  Repeat,
  StopCircle,
  Play,
  ListPlus,
  Target,
} from "lucide-react";

// ---------------------------------------------------------------------------
// Design tokens (kept out of Tailwind's default palette on purpose so the
// panel reads as one deliberate dark instrument, not a generic "dark mode")
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

// ---------------------------------------------------------------------------
// 5-DOF Arm configuration
// J1 = ฐาน (หมุน 360° N20 AB Encoder, soft-limit -180..+180)
// J2 = ไหล่ (ขึ้น-ลง)
// J3 = ข้อศอก (ขึ้น-ลง)
// J4 = ข้อมือ (ขึ้น-ลง)
// J5 = ปลายจับ / Gripper (เปิด-ปิด symmetric, 0=ปิดสนิท..100=เปิดสุด %)
// ---------------------------------------------------------------------------
const JOINTS = [
  { key: "j1", label: "J1", sub: "ฐาน (หมุน)", min: -180, max: 180, unit: "deg" },
  { key: "j2", label: "J2", sub: "ไหล่", min: -90, max: 90, unit: "deg" },
  { key: "j3", label: "J3", sub: "ข้อศอก", min: -135, max: 135, unit: "deg" },
  { key: "j4", label: "J4", sub: "ข้อมือ", min: -135, max: 135, unit: "deg" },
  { key: "j5", label: "J5", sub: "ปลายจับ (เปิด/ปิด)", min: 0, max: 100, unit: "%" },
];

const HOME = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0 };

const THAI_MONTHS = [
  "มกราคม","กุมภาพันธ์","มีนาคม","เมษายน","พฤษภาคม","มิถุนายน",
  "กรกฎาคม","สิงหาคม","กันยายน","ตุลาคม","พฤศจิกายน","ธันวาคม",
];

const NAV_ITEMS = [
  { key: "manual", icon: Gamepad2, title: "Manual Control", sub: "ควบคุมแบบ Manual" },
  { key: "record", icon: PlayCircle, title: "Record & Playback", sub: "บันทึกและเล่นท่าทาง" },
  { key: "status", icon: Activity, title: "System Status", sub: "สถานะระบบ" },
  { key: "about", icon: Info, title: "About Program", sub: "เกี่ยวกับโปรแกรม" },
];

// ---------------------------------------------------------------------------
// Small UI primitives
// ---------------------------------------------------------------------------
function Panel({ children, style, className = "" }) {
  return (
    <div
      className={`rounded-2xl ${className}`}
      style={{ background: C.panel, border: `1px solid ${C.border}`, ...style }}
    >
      {children}
    </div>
  );
}

function PanelHeader({ title, sub }) {
  return (
    <div className="px-5 pt-4 pb-3">
      <div className="text-[15px] font-semibold" style={{ color: C.text }}>{title}</div>
      {sub && <div className="text-xs mt-0.5" style={{ color: C.sub }}>{sub}</div>}
    </div>
  );
}

function ReadoutField({ label, value, unit }) {
  return (
    <div>
      <div className="text-xs mb-1.5" style={{ color: C.sub }}>{label}</div>
      <div
        className="flex items-center justify-between rounded-xl px-3 py-2.5"
        style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}
      >
        <span className="font-mono text-[15px] tabular-nums" style={{ color: C.text }}>
          {value}
        </span>
        <span className="text-xs" style={{ color: C.subDim }}>{unit}</span>
      </div>
    </div>
  );
}

function StatusRow({ label, value, valueColor }) {
  return (
    <div className="flex items-center justify-between py-2.5" style={{ borderBottom: `1px solid ${C.borderSoft}` }}>
      <span className="text-xs" style={{ color: C.sub }}>{label}</span>
      <span className="text-sm font-medium tabular-nums" style={{ color: valueColor || C.text }}>{value}</span>
    </div>
  );
}

// ---------------------------------------------------------------------------
// 3D arm scene — plain three.js (no OrbitControls in this r128 build), with a
// small hand-rolled orbit/zoom/pan controller driven off pointer events.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 3D Arm Scene — โหลดโมเดล GLTF จาก Blender เท่านั้น แล้วอ่าน Joint hierarchy
// จากชื่อ object ที่ตั้งไว้ในไฟล์โมเดล (ไม่มี procedural fallback อีกต่อไป)
// ---------------------------------------------------------------------------
function useArmScene(containerRef, joints, wireframe) {
  const sceneRef = useRef(null);
  const [modelReady, setModelReady] = useState(false);
  const [modelError, setModelError] = useState(false);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return undefined;

    // ---- Renderer / Scene / Camera ----
    const scene = new THREE.Scene();

    // Gradient background (dark navy) — ใช้ vertex colors บน mesh พื้นหลัง
    // (ไม่เกี่ยวข้องกับตัวแขนกล เป็นแค่ฉากหลังของวิวพอร์ต)
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

    function onPointerDown(e) {
      controls.dragging = true;
      controls.panMode = e.shiftKey || controls.panMode;
      controls.lastX = e.clientX;
      controls.lastY = e.clientY;
      renderer.domElement.style.cursor = "grabbing";
    }
    function onPointerMove(e) {
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

    let raf;
    function tick() {
      applyCamera();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(tick);
    }
    tick();

    // ---- ไม่มี procedural arm อีกต่อไป — รอโหลดโมเดลจาก Blender (GLTF) เท่านั้น ----
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
      ready: false,
    };

    // ---- โหลดโมเดล GLTF จาก public/robotarm.glb ----
    // ใช้ GLTFLoader ที่ bundle มากับแอป (import ไว้ด้านบนของไฟล์) แทนการโหลดจาก CDN
    // ตอน runtime เพื่อให้ทำงานได้แม้ไม่มีอินเทอร์เน็ต (สำคัญมากสำหรับ Electron build)
    // โมเดลนี้เป็นแหล่งเดียวของรูปทรงแขนกล — ไม่มีการสร้างชิ้นส่วนขึ้นเองด้วยโค้ดอีกแล้ว
    const loader = new GLTFLoader();

    loader.load(
      robotModel,
      (gltf) => {
        console.log("GLTF Loaded");
        buildFromGLTF(gltf);
      },
      (xhr) => {
        console.log("Progress", xhr.loaded, xhr.total);
      },
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
      console.log("===== MODEL OBJECTS =====");
      
      model.traverse((obj) => {
        console.log(obj.name, obj.type);
      });
      s.baseGroup = model.getObjectByName("Base");
      s.shoulder = model.getObjectByName("ArmJ2");
      s.elbow = model.getObjectByName("ArmJ3");
      s.wrist = model.getObjectByName("ArmGripper");
      s.gripperGroup = model.getObjectByName("FingerBase");
      s.fingerL = model.getObjectByName("Left_Finger");
      s.fingerR = model.getObjectByName("Right_Finger");
      s.endEffector = s.gripperGroup;

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

  // update joint rotations whenever angles change; return computed pose
  const [pose, setPose] = useState({ x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 });

  useEffect(() => {
    const s = sceneRef.current;
    if (!s || !s.ready) return; // โมเดล GLTF ยังโหลดไม่เสร็จ — ยังไม่มีอะไรให้ขยับ
    const d = THREE.MathUtils.degToRad;

    // J1 — ฐาน หมุนรอบแกน Y (yaw)
    s.baseGroup.rotation.y = d(joints.j1);
    // J2 — ไหล่ pitch รอบแกน Z
    s.shoulder.rotation.z = d(-joints.j2);
    // J3 — ข้อศอก pitch รอบแกน Z
    s.elbow.rotation.z = d(-joints.j3);
    // J4 — ข้อมือ pitch รอบแกน Z
    s.wrist.rotation.z = d(-joints.j4);
    // J5 — ปลายจับ symmetric: แปลง 0..100% → กางนิ้วออก 0..0.13 units
    const fingerSpread = (joints.j5 / 100) * 0.13;
    s.fingerL.position.x = -(0.055 + fingerSpread);
    s.fingerR.position.x = +(0.055 + fingerSpread);

    s.baseGroup.updateMatrixWorld(true);

    const scale = 300; // model units -> mm
    const pos = new THREE.Vector3();
    s.endEffector.getWorldPosition(pos);
    const quat = new THREE.Quaternion();
    s.endEffector.getWorldQuaternion(quat);
    const euler = new THREE.Euler().setFromQuaternion(quat, "XYZ");

    setPose({
      x: pos.x * scale,
      y: pos.z * scale,
      z: pos.y * scale,
      roll:  THREE.MathUtils.radToDeg(euler.x),
      pitch: THREE.MathUtils.radToDeg(euler.z),
      yaw:   THREE.MathUtils.radToDeg(euler.y),
    });
  }, [joints, modelReady]);

  useEffect(() => {
    const s = sceneRef.current;
    if (!s || !s.ready) return;
    s.allMeshes.forEach((m) => { m.material.wireframe = wireframe; });
  }, [wireframe, modelReady]);

  const resetView = useCallback(() => {
    const s = sceneRef.current;
    if (!s) return;
    s.controls.azimuth = Math.PI * 0.28;
    s.controls.elevation = 0.5;
    s.controls.radius = 4.6;
    s.controls.target.set(0, 0.9, 0);
    s.controls.panMode = false;
  }, []);
  const zoom = useCallback((dir) => {
    const s = sceneRef.current;
    if (!s) return;
    s.controls.radius = Math.max(2, Math.min(10, s.controls.radius + dir * 0.5));
  }, []);
  const setPanMode = useCallback((v) => {
    const s = sceneRef.current;
    if (s) s.controls.panMode = v;
  }, []);

  return { pose, resetView, zoom, setPanMode, modelReady, modelError };
}


// ---------------------------------------------------------------------------
// Joint slider block
// ---------------------------------------------------------------------------
function JointControl({ label, sub, value, onChange, disabled, min = -180, max = 180, unit = "deg" }) {
  const isGripper = unit === "%";
  const [stepInput, setStepInput] = React.useState(isGripper ? "1" : "1");
  const clamp = (v) => Math.max(min, Math.min(max, v));
  const stepVal = Math.max(0.1, parseFloat(stepInput) || 1);

  return (
    <div
      className="rounded-xl px-3.5 py-3.5"
      style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, opacity: disabled ? 0.45 : 1 }}
    >
      <div className="flex items-baseline justify-between mb-2.5">
        <span className="text-xs font-medium" style={{ color: C.sub }}>
          {label} <span style={{ color: C.subDim }}>({sub})</span>
        </span>
      </div>
      <div
        className="flex items-center justify-between rounded-lg px-2.5 py-1.5 mb-3"
        style={{ background: C.panel, border: `1px solid ${C.borderSoft}` }}
      >
        <span className="font-mono text-base tabular-nums" style={{ color: C.text }}>
          {isGripper ? value.toFixed(0) : value.toFixed(1)}
        </span>
        <span className="text-xs" style={{ color: C.subDim }}>{unit}</span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={isGripper ? 1 : 0.5}
        value={value}
        disabled={disabled}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="w-full accent-blue-500"
        style={{ accentColor: isGripper ? C.green : C.accent }}
      />
      <div className="flex items-center justify-between text-[10px] mt-1" style={{ color: C.subDim }}>
        <span>{isGripper ? "ปิด 0%" : `${min}°`}</span>
        <span>{isGripper ? "เปิด 100%" : `${max}°`}</span>
      </div>
      <div className="flex items-center gap-2 mt-3">
        <button
          disabled={disabled}
          onClick={() => onChange(clamp(value - stepVal))}
          className="flex-1 h-7 rounded-md flex items-center justify-center transition-colors"
          style={{ background: C.panel, border: `1px solid ${C.borderSoft}`, color: C.sub }}
        >
          <Minus size={13} />
        </button>
        <input
          type="number"
          value={stepInput}
          min={0.1}
          max={isGripper ? 100 : 180}
          disabled={disabled}
          onChange={(e) => setStepInput(e.target.value)}
          onBlur={(e) => {
            const v = parseFloat(e.target.value);
            if (!isNaN(v) && v > 0) setStepInput(String(v));
            else setStepInput(isGripper ? "1" : "1");
          }}
          className="flex-1 h-7 rounded-md text-center text-[11px] outline-none"
          style={{
            background: C.panel,
            border: `1px solid ${C.accent}`,
            color: C.text,
            width: 0,
          }}
        />
        <span className="text-[10px] shrink-0" style={{ color: C.subDim }}>{unit}</span>
        <button
          disabled={disabled}
          onClick={() => onChange(clamp(value + stepVal))}
          className="flex-1 h-7 rounded-md flex items-center justify-center transition-colors"
          style={{ background: C.panel, border: `1px solid ${C.borderSoft}`, color: C.sub }}
        >
          <Plus size={13} />
        </button>
      </div>
    </div>
  );
}

// ---------------------------------------------------------------------------
// Main component
// ---------------------------------------------------------------------------
const WAYPOINTS_STORAGE_KEY = "robotArm.waypoints.v1";

export default function RoboticArmControl() {
  const [activeTab, setActiveTab] = useState("manual");
  const [joints, setJoints] = useState(HOME);
  const [speed, setSpeed] = useState(60);
  const [mode, setMode] = useState("normal");
  const [wireframe, setWireframe] = useState(false);
  const [estopped, setEstopped] = useState(false);
  const [toast, setToast] = useState("");
  const [ports, setPorts] = useState(["COM3"]);
  const [selectedPort, setSelectedPort] = useState("COM3");
  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);

  // ---------------- Record & Playback -------------------------------------
  const [waypoints, setWaypoints] = useState(() => {
    try {
      const raw = window.localStorage?.getItem(WAYPOINTS_STORAGE_KEY);
      return raw ? JSON.parse(raw) : [];
    } catch {
      return [];
    }
  });
  const [waypointName, setWaypointName] = useState("");
  const [playing, setPlaying] = useState(false);
  const [playIndex, setPlayIndex] = useState(-1);
  const [playProgress, setPlayProgress] = useState(0); // 0..1 within current step
  const [loopPlayback, setLoopPlayback] = useState(false);
  const cancelPlaybackRef = useRef(false);
  const loopRef = useRef(false);
  const speedRef = useRef(60);

  useEffect(() => { loopRef.current = loopPlayback; }, [loopPlayback]);
  useEffect(() => { speedRef.current = speed; }, [speed]);

  useEffect(() => {
    try {
      window.localStorage?.setItem(WAYPOINTS_STORAGE_KEY, JSON.stringify(waypoints));
    } catch {
      /* ignore quota / privacy-mode errors */
    }
  }, [waypoints]);

  // ถ้ารันในแอป Electron จะมี window.electronAPI ให้ใช้จริง (ดู electron/preload.js)
  // ถ้ารันในเบราว์เซอร์ธรรมดา (เช่นตอน dev ด้วย `npm run dev`) จะ mock พอร์ตไว้ให้แทน
  useEffect(() => {
    if (window.electronAPI?.listPorts) {
      window.electronAPI.listPorts().then((list) => {
        if (list?.length) {
          setPorts(list);
          setSelectedPort(list[0]);
        }
      }).catch(() => {});
    }
  }, []);

  // เชื่อมต่อจริงผ่าน window.electronAPI แล้วตรวจสอบสถานะกลับมาแทนการเดา/ตั้งค่าเอง
  // ถ้าไม่ได้รันในแอป Electron (เช่น `npm run dev` ในเบราว์เซอร์เฉยๆ) จะไม่มีทางเชื่อมต่อ
  // ฮาร์ดแวร์จริงได้ สถานะจะแสดง "ไม่ได้เชื่อมต่อ" ตามจริง แทนที่จะหลอกว่าเชื่อมต่อแล้ว
  const attemptConnect = useCallback(async (portName) => {
    if (!window.electronAPI?.connectPort) {
      setConnected(false);
      return;
    }
    setConnecting(true);
    try {
      const res = await window.electronAPI.connectPort(portName);
      setConnected(!!res?.ok);
    } catch {
      setConnected(false);
    } finally {
      setConnecting(false);
    }
  }, []);

  const checkConnectionStatus = useCallback(async () => {
    if (!window.electronAPI?.getStatus) {
      setConnected(false);
      return;
    }
    try {
      const res = await window.electronAPI.getStatus();
      setConnected(!!res?.connected);
    } catch {
      setConnected(false);
    }
  }, []);

  // ปุ่มสถานะไม่สามารถกดสลับเองได้อีกต่อไป — กดได้แค่ตอน "ไม่ได้เชื่อมต่อ" เพื่อลองเชื่อมต่อใหม่
  const handleRetryConnection = () => {
    if (!connected && !connecting) attemptConnect(selectedPort);
  };

  // เชื่อมต่ออัตโนมัติเมื่อเปลี่ยนพอร์ต แล้วตรวจสอบสถานะจริงซ้ำเป็นระยะ เพื่อให้ตัวบ่งชี้
  // สะท้อนการเชื่อมต่อฮาร์ดแวร์จริง ไม่ใช่แค่ค่าที่เคยตั้งไว้ครั้งเดียว
  useEffect(() => {
    attemptConnect(selectedPort);
  }, [selectedPort, attemptConnect]);

  useEffect(() => {
    const id = setInterval(checkConnectionStatus, 4000);
    return () => clearInterval(id);
  }, [checkConnectionStatus]);

  // ฟังข้อความจาก ESP32 (ack / telemetry / disconnected) ที่ main.js forward มาให้
  // แบบ real-time ผ่าน IPC — ตอนนี้ใช้แค่รีเช็คสถานะทันทีเมื่อพอร์ตหลุดกะทันหัน
  useEffect(() => {
    if (!window.electronAPI?.onSerialData) return undefined;
    const unsubscribe = window.electronAPI.onSerialData((msg) => {
      if (msg?.type === "disconnected") {
        setConnected(false);
      }
      // msg?.type === "ack" หรือ "telemetry" — จุดต่อยอดสำหรับอนาคต เช่น
      // อัปเดตแรงดันไฟ/อุณหภูมิในแท็บ System Status จากข้อมูลจริงของบอร์ด
    });
    return unsubscribe;
  }, []);

  // ส่งมุมข้อต่อปัจจุบันออกไปยัง ESP32 จริงผ่าน Serial (โปรโตคอล JSON บรรทัดเดียว
  // ดูรายละเอียดที่คอมเมนต์ใน electron/main.js)
  const handleSendToBoard = async () => {
    if (!window.electronAPI?.sendJointAngles) {
      showToast("ฟีเจอร์นี้ใช้ได้เฉพาะเมื่อรันผ่านแอป Electron เท่านั้น");
      return;
    }
    if (!connected) {
      showToast("ยังไม่ได้เชื่อมต่อพอร์ต — เชื่อมต่อก่อนส่งค่า");
      return;
    }
    try {
      const res = await window.electronAPI.sendJointAngles(joints);
      if (res?.ok) {
        showToast(res.mock ? "ส่งค่าแล้ว (โหมดจำลอง — ยังไม่มีฮาร์ดแวร์จริง)" : "ส่งค่าไปยังบอร์ดแล้ว");
      } else {
        showToast(`ส่งค่าไม่สำเร็จ: ${res?.error ?? "ไม่ทราบสาเหตุ"}`);
      }
    } catch (err) {
      showToast(`ส่งค่าไม่สำเร็จ: ${err.message}`);
    }
  };
  const [now, setNow] = useState(new Date());
  const [telemetry, setTelemetry] = useState({ voltage: 12.0, temp: 36.2, fps: 60 });

  const containerRef = useRef(null);
  const { pose, resetView, zoom, setPanMode, modelReady, modelError } = useArmScene(containerRef, joints, wireframe);

  useEffect(() => {
    const t = setInterval(() => setNow(new Date()), 1000);
    return () => clearInterval(t);
  }, []);

  useEffect(() => {
    const t = setInterval(() => {
      setTelemetry((prev) => ({
        voltage: +(11.9 + Math.random() * 0.3).toFixed(1),
        temp: +(35.5 + Math.random() * 1.6).toFixed(1),
        fps: 58 + Math.floor(Math.random() * 3),
      }));
    }, 1800);
    return () => clearInterval(t);
  }, []);

  const setJoint = (key, v) => {
    if (estopped) return;
    setJoints((prev) => ({ ...prev, [key]: v }));
  };

  const showToast = (msg) => {
    setToast(msg);
    setTimeout(() => setToast(""), 1800);
  };

  // add the current joint pose as a new waypoint in the Record & Playback list
  const addWaypoint = useCallback((customName) => {
    setWaypoints((prev) => {
      const name = (customName && customName.trim()) || `ท่าที่ ${prev.length + 1}`;
      const wp = {
        id: `wp-${Date.now()}-${Math.random().toString(36).slice(2, 7)}`,
        name,
        joints: { ...joints },
        createdAt: Date.now(),
      };
      return [...prev, wp];
    });
    setWaypointName("");
  }, [joints]);

  const handleSave = () => {
    addWaypoint(waypointName);
    showToast("บันทึกตำแหน่งนี้ลงใน Record & Playback แล้ว");
  };
  const handleHome = () => {
    if (estopped) return;
    setJoints(HOME);
    showToast("กลับสู่ตำแหน่งเริ่มต้นแล้ว");
  };
  const handleStop = () => {
    if (!estopped && playing) {
      cancelPlaybackRef.current = true;
      setPlaying(false);
      setPlayIndex(-1);
      setPlayProgress(0);
    }
    setEstopped((v) => !v);
    showToast(estopped ? "กลับมาทำงานตามปกติ" : "หยุดการทำงานฉุกเฉิน");
  };

  const deleteWaypoint = (id) => {
    setWaypoints((prev) => prev.filter((w) => w.id !== id));
  };

  const moveWaypoint = (id, dir) => {
    setWaypoints((prev) => {
      const idx = prev.findIndex((w) => w.id === id);
      const next = idx + dir;
      if (idx < 0 || next < 0 || next >= prev.length) return prev;
      const copy = [...prev];
      [copy[idx], copy[next]] = [copy[next], copy[idx]];
      return copy;
    });
  };

  const jumpToWaypoint = (wp) => {
    if (estopped) return;
    setJoints(wp.joints);
    showToast(`ไปยัง "${wp.name}" แล้ว`);
  };

  const stopPlayback = () => {
    cancelPlaybackRef.current = true;
    setPlaying(false);
    setPlayIndex(-1);
    setPlayProgress(0);
    showToast("หยุดการเล่นท่าทาง");
  };

  const easeInOut = (p) => (p < 0.5 ? 2 * p * p : 1 - Math.pow(-2 * p + 2, 2) / 2);

  // plays a sequence of waypoints in order, interpolating smoothly between
  // each recorded pose; honors the global speed slider and the loop toggle
  const playSequence = useCallback(async (list, startIndex = 0) => {
    if (!list.length) {
      showToast("ยังไม่มีท่าทางที่บันทึกไว้");
      return;
    }
    if (estopped) {
      showToast("ยกเลิกหยุดฉุกเฉินก่อนเล่นท่าทาง");
      return;
    }
    cancelPlaybackRef.current = false;
    setPlaying(true);
    setActiveTab("manual");

    let current = { ...joints };
    let keepGoing = true;
    while (keepGoing && !cancelPlaybackRef.current) {
      for (let i = startIndex; i < list.length; i++) {
        if (cancelPlaybackRef.current) break;
        setPlayIndex(i);
        setPlayProgress(0);
        const target = list[i].joints;
        const spd = Math.max(10, speedRef.current);
        const duration = Math.max(350, 2400 * (100 / spd));
        const t0 = performance.now();
        // eslint-disable-next-line no-await-in-loop
        await new Promise((resolve) => {
          function step(now) {
            if (cancelPlaybackRef.current) { resolve(); return; }
            const elapsed = now - t0;
            const p = Math.min(1, elapsed / duration);
            const eased = easeInOut(p);
            const next = {};
            JOINTS.forEach((j) => {
              next[j.key] = current[j.key] + (target[j.key] - current[j.key]) * eased;
            });
            setJoints(next);
            setPlayProgress(p);
            if (p < 1) requestAnimationFrame(step);
            else resolve();
          }
          requestAnimationFrame(step);
        });
        current = target;
        if (cancelPlaybackRef.current) break;
        // brief settle pause at each waypoint
        // eslint-disable-next-line no-await-in-loop
        await new Promise((r) => setTimeout(r, 300));
      }
      startIndex = 0;
      keepGoing = loopRef.current && !cancelPlaybackRef.current;
    }

    setPlaying(false);
    setPlayIndex(-1);
    setPlayProgress(0);
  }, [joints, estopped]);

  const playAll = () => playSequence(waypoints, 0);
  const playFrom = (id) => {
    const idx = waypoints.findIndex((w) => w.id === id);
    if (idx >= 0) playSequence(waypoints, idx);
  };

  const dateStr = `วันที่ ${now.getDate()} ${THAI_MONTHS[now.getMonth()]} ${now.getFullYear() + 543}`;
  const timeStr = `เวลา ${now.toLocaleTimeString("th-TH", { hour12: false })}`;

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
            5-DOF Robotic Arm Control
          </span>
        </div>

        <div className="flex items-center gap-3">
          <button
            onClick={handleRetryConnection}
            disabled={connected || connecting}
            title={!connected ? "คลิกเพื่อลองเชื่อมต่อใหม่" : undefined}
            className="flex items-center gap-2 px-3 py-1.5 rounded-full text-xs"
            style={{
              background: !connected ? C.redSoft : estopped ? C.redSoft : "rgba(34,197,94,0.12)",
              cursor: !connected && !connecting ? "pointer" : "default",
            }}
          >
            <span
              className="w-1.5 h-1.5 rounded-full"
              style={{ background: !connected ? C.red : estopped ? C.red : C.green }}
            />
            <span style={{ color: !connected ? C.red : estopped ? C.red : C.green }}>
              {connecting ? "กำลังเชื่อมต่อ..." : !connected ? "ไม่ได้เชื่อมต่อ" : estopped ? "หยุดฉุกเฉิน" : "เชื่อมต่อแล้ว"}
            </span>
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
        </div>
      </div>

      <div className="flex flex-1 min-h-0">
        {/* ---------------- Sidebar ---------------- */}
        <div className="w-56 shrink-0 px-3 py-4 flex flex-col gap-2" style={{ background: C.panel, borderRight: `1px solid ${C.border}` }}>
          {NAV_ITEMS.map((item) => {
            const active = activeTab === item.key;
            const Icon = item.icon;
            return (
              <button
                key={item.key}
                onClick={() => setActiveTab(item.key)}
                className="flex items-center gap-3 rounded-xl px-3.5 py-3 text-left transition-colors"
                style={{
                  background: active ? C.accent : "transparent",
                  border: `1px solid ${active ? C.accent : "transparent"}`,
                }}
              >
                <Icon size={18} color={active ? "#fff" : C.sub} />
                <div>
                  <div className="text-sm font-medium" style={{ color: active ? "#fff" : C.text }}>
                    {item.title}
                  </div>
                  <div className="text-[11px]" style={{ color: active ? "rgba(255,255,255,0.75)" : C.subDim }}>
                    {item.sub}
                  </div>
                </div>
              </button>
            );
          })}
        </div>

        {/* ---------------- Main ---------------- */}
        <div className="flex-1 min-w-0 p-5 overflow-auto">
          <div style={{ display: activeTab === "manual" ? "block" : "none" }}>
            <>
              <div className="mb-4">
                <h1 className="text-xl font-semibold" style={{ color: C.text }}>Manual Control</h1>
                <p className="text-xs mt-0.5" style={{ color: C.sub }}>ควบคุมแขนกลแบบเรียลไทม์</p>
              </div>

              <div className="flex gap-4 items-stretch flex-wrap xl:flex-nowrap">
                {/* left column */}
                <div className="flex-1 min-w-[380px] flex flex-col gap-4">
                  <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
                    <Panel>
                      <PanelHeader title="ตำแหน่งปลายแขน (Cartesian)" />
                      <div className="px-5 pb-5 flex flex-col gap-3">
                        <ReadoutField label="X" value={pose.x.toFixed(1)} unit="mm" />
                        <ReadoutField label="Y" value={pose.y.toFixed(1)} unit="mm" />
                        <ReadoutField label="Z" value={pose.z.toFixed(1)} unit="mm" />
                      </div>
                    </Panel>
                    <Panel>
                      <PanelHeader title="การวางแนวปลายแขน (Orientation)" />
                      <div className="px-5 pb-5 flex flex-col gap-3">
                        <ReadoutField label="Roll (Rx)" value={pose.roll.toFixed(1)} unit="deg" />
                        <ReadoutField label="Pitch (Ry)" value={pose.pitch.toFixed(1)} unit="deg" />
                        <ReadoutField label="Yaw (Rz)" value={pose.yaw.toFixed(1)} unit="deg" />
                      </div>
                    </Panel>
                  </div>

                  <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
                    <Panel>
                      <PanelHeader title="ความเร็วการเคลื่อนที่" />
                      <div className="px-5 pb-5 flex items-center gap-3">
                        <input
                          type="range" min={0} max={100} value={speed}
                          onChange={(e) => setSpeed(parseInt(e.target.value, 10))}
                          className="flex-1" style={{ accentColor: C.accent }}
                        />
                        <div
                          className="w-14 h-9 rounded-lg flex items-center justify-center font-mono text-sm"
                          style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text }}
                        >
                          {speed}%
                        </div>
                      </div>
                    </Panel>
                    <Panel>
                      <PanelHeader title="โหมดการทำงาน" />
                      <div className="px-5 pb-5">
                        <div className="relative">
                          <select
                            value={mode}
                            onChange={(e) => setMode(e.target.value)}
                            className="w-full appearance-none rounded-lg px-3 py-2.5 text-sm outline-none"
                            style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text }}
                          >
                            <option value="normal">โหมดปกติ</option>
                            <option value="smooth">โหมดนุ่มนวล</option>
                            <option value="fast">โหมดเร็ว</option>
                            <option value="precise">โหมดละเอียด</option>
                          </select>
                          <ChevronDown size={14} className="pointer-events-none absolute right-3 top-1/2 -translate-y-1/2" color={C.subDim} />
                        </div>
                      </div>
                    </Panel>
                  </div>
                </div>

                {/* 3D viewport */}
                <Panel className="flex-1 min-w-[380px] flex flex-col" style={{ minHeight: 380 }}>
                  <div className="flex items-center justify-between px-5 pt-4 pb-2">
                    <span className="text-[15px] font-semibold" style={{ color: C.text }}>แสดงโมเดล 3D</span>
                    <div className="flex items-center gap-1">
                      {[
                        { icon: Move, onClick: () => setPanMode(true), title: "ลาก (Pan)" },
                        { icon: Crosshair, onClick: resetView, title: "รีเซ็ตมุมมอง" },
                        { icon: ZoomOut, onClick: () => zoom(1), title: "ซูมออก" },
                        { icon: ZoomIn, onClick: () => zoom(-1), title: "ซูมเข้า" },
                        { icon: BoxIcon, onClick: () => setWireframe((v) => !v), title: "โครงลวด" },
                      ].map((b, i) => (
                        <button
                          key={i}
                          title={b.title}
                          onClick={b.onClick}
                          className="w-7 h-7 rounded-md flex items-center justify-center"
                          style={{ color: C.sub, background: C.panelAlt }}
                        >
                          <b.icon size={14} />
                        </button>
                      ))}
                    </div>
                  </div>
                  <div className="flex-1 mx-3 mb-3 rounded-xl overflow-hidden relative" style={{ minHeight: 300 }}>
                    <div ref={containerRef} className="absolute inset-0" />
                    {!modelReady && !modelError && (
                      <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                        <span className="text-xs" style={{ color: C.subDim }}>กำลังโหลดโมเดล 3D...</span>
                      </div>
                    )}
                    {modelError && (
                      <div className="absolute inset-0 flex items-center justify-center pointer-events-none px-6 text-center">
                        <span className="text-xs" style={{ color: C.red }}>
                          โหลดโมเดล /robotarm.glb ไม่สำเร็จ — ตรวจสอบว่าไฟล์อยู่ใน public/ และมีชื่อ object ตรงกับที่กำหนด
                        </span>
                      </div>
                    )}
                  </div>
                </Panel>
              </div>

              {/* joint control + status row */}
              <div className="flex gap-4 mt-4 items-start flex-wrap xl:flex-nowrap">
                <Panel className="flex-1 min-w-[500px]">
                  <PanelHeader title="ควบคุมมุมข้อต่อ (Joint Control)" />
                  <div className="px-5 pb-5">
                    <div className="grid gap-3" style={{ gridTemplateColumns: "repeat(auto-fit, minmax(150px, 1fr))" }}>
                      {JOINTS.map((j) => (
                        <JointControl
                          key={j.key}
                          label={j.label}
                          sub={j.sub}
                          min={j.min}
                          max={j.max}
                          unit={j.unit}
                          value={joints[j.key]}
                          disabled={estopped}
                          onChange={(v) => setJoint(j.key, v)}
                        />
                      ))}
                    </div>

                    <div className="flex flex-wrap gap-3 mt-4">
                      <button
                        onClick={handleSendToBoard}
                        disabled={estopped || !connected}
                        className="flex-1 min-w-[160px] h-11 rounded-xl flex items-center justify-center gap-2 text-sm font-medium"
                        style={{ background: C.green, color: "#fff", opacity: estopped || !connected ? 0.5 : 1 }}
                        title={!connected ? "ยังไม่ได้เชื่อมต่อพอร์ต" : "ส่งมุมข้อต่อปัจจุบันไปยัง ESP32"}
                      >
                        <Send size={15} /> ส่งค่าไปยังบอร์ด
                      </button>
                      <button
                        onClick={handleSave}
                        disabled={estopped}
                        className="flex-1 min-w-[160px] h-11 rounded-xl flex items-center justify-center gap-2 text-sm font-medium transition-colors"
                        style={{ background: C.accent, color: "#fff", opacity: estopped ? 0.5 : 1 }}
                      >
                        <Save size={15} /> บันทึกตำแหน่งนี้
                      </button>
                      <button
                        onClick={handleHome}
                        disabled={estopped}
                        className="flex-1 min-w-[160px] h-11 rounded-xl flex items-center justify-center gap-2 text-sm font-medium"
                        style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text, opacity: estopped ? 0.5 : 1 }}
                      >
                        <RotateCcw size={15} /> กลับตำแหน่งเริ่มต้น
                      </button>
                      <button
                        onClick={handleStop}
                        className="flex-1 min-w-[160px] h-11 rounded-xl flex items-center justify-center gap-2 text-sm font-medium"
                        style={{ background: estopped ? C.green : C.red, color: "#fff" }}
                      >
                        <Square size={14} fill="#fff" /> {estopped ? "เริ่มการทำงาน" : "หยุดการทำงาน"}
                      </button>
                    </div>
                  </div>
                </Panel>

                <Panel className="w-full xl:w-72 shrink-0">
                  <PanelHeader title="สถานะการเชื่อมต่อ" />
                  <div className="px-5 pb-5">
                    <div className="flex items-center justify-between py-2.5" style={{ borderBottom: `1px solid ${C.borderSoft}` }}>
                      <span className="text-xs" style={{ color: C.sub }}>สถานะ</span>
                      <span className="flex items-center gap-1.5 text-sm font-medium"
                        style={{ color: !connected ? C.red : estopped ? C.red : C.green }}>
                        <span className="w-1.5 h-1.5 rounded-full"
                          style={{ background: !connected ? C.red : estopped ? C.red : C.green }} />
                        {connecting ? "กำลังเชื่อมต่อ..." : !connected ? "ไม่ได้เชื่อมต่อ" : estopped ? "หยุดฉุกเฉิน" : "เชื่อมต่อแล้ว"}
                      </span>
                    </div>
                    <StatusRow label="พอร์ต" value={selectedPort} />
                    <StatusRow label="อัตราการส่งข้อมูล" value="115200 bps" />
                    <StatusRow label="แรงดันไฟเลี้ยง" value={`${telemetry.voltage.toFixed(1)} V`} />
                    <StatusRow label="อุณหภูมิระบบ" value={`${telemetry.temp.toFixed(1)} °C`} />
                    <div className="flex items-center justify-between pt-2.5">
                      <span className="text-xs" style={{ color: C.sub }}>FPS</span>
                      <span className="text-sm font-medium tabular-nums" style={{ color: C.text }}>{telemetry.fps}</span>
                    </div>
                  </div>
                </Panel>
              </div>
            </>
          </div>

          <div style={{ display: activeTab === "record" ? "block" : "none" }}>
            <>
              <div className="mb-4">
                <h1 className="text-xl font-semibold" style={{ color: C.text }}>Record &amp; Playback</h1>
                <p className="text-xs mt-0.5" style={{ color: C.sub }}>บันทึกและเล่นท่าทางอัตโนมัติ</p>
              </div>

              <div className="flex gap-4 items-start flex-wrap xl:flex-nowrap">
                {/* left column: capture + playback settings */}
                <div className="w-full xl:w-80 shrink-0 flex flex-col gap-4">
                  <Panel>
                    <PanelHeader title="บันทึกท่าทางปัจจุบัน" sub="บันทึกมุมข้อต่อทั้ง 6 แกน ณ ตำแหน่งปัจจุบัน" />
                    <div className="px-5 pb-5 flex flex-col gap-3">
                      <input
                        value={waypointName}
                        onChange={(e) => setWaypointName(e.target.value)}
                        placeholder={`ท่าที่ ${waypoints.length + 1}`}
                        className="w-full rounded-lg px-3 py-2.5 text-sm outline-none"
                        style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text }}
                      />
                      <button
                        onClick={() => { addWaypoint(waypointName); showToast("เพิ่มท่าทางลงรายการแล้ว"); }}
                        disabled={estopped}
                        className="h-11 rounded-xl flex items-center justify-center gap-2 text-sm font-medium"
                        style={{ background: C.accent, color: "#fff", opacity: estopped ? 0.5 : 1 }}
                      >
                        <ListPlus size={15} /> เพิ่มเป็นท่าทางใหม่
                      </button>
                      <p className="text-[11px] leading-relaxed" style={{ color: C.subDim }}>
                        เคล็ดลับ: ปรับมุมข้อต่อในแท็บ Manual Control แล้วกด &quot;บันทึกตำแหน่งนี้&quot; ก็จะถูกเพิ่มเข้ามาที่นี่โดยอัตโนมัติ
                      </p>
                    </div>
                  </Panel>

                  <Panel>
                    <PanelHeader title="การเล่นท่าทาง" />
                    <div className="px-5 pb-5 flex flex-col gap-3">
                      <div className="flex items-center justify-between rounded-lg px-3 py-2.5" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}>
                        <span className="text-xs" style={{ color: C.sub }}>ความเร็วเล่น (ใช้ค่าจาก Manual Control)</span>
                        <span className="text-sm font-mono" style={{ color: C.text }}>{speed}%</span>
                      </div>
                      <button
                        onClick={() => setLoopPlayback((v) => !v)}
                        className="flex items-center justify-between rounded-lg px-3 py-2.5 text-sm"
                        style={{
                          background: loopPlayback ? C.accentSoft : C.panelAlt,
                          border: `1px solid ${loopPlayback ? C.accent : C.borderSoft}`,
                          color: loopPlayback ? C.accent : C.text,
                        }}
                      >
                        <span className="flex items-center gap-2"><Repeat size={15} /> เล่นวนซ้ำ</span>
                        <span
                          className="w-9 h-5 rounded-full relative transition-colors"
                          style={{ background: loopPlayback ? C.accent : C.track }}
                        >
                          <span
                            className="absolute top-0.5 w-4 h-4 rounded-full bg-white transition-all"
                            style={{ left: loopPlayback ? 18 : 2 }}
                          />
                        </span>
                      </button>

                      {!playing ? (
                        <button
                          onClick={playAll}
                          disabled={estopped || waypoints.length === 0}
                          className="h-11 rounded-xl flex items-center justify-center gap-2 text-sm font-medium"
                          style={{ background: C.green, color: "#fff", opacity: estopped || waypoints.length === 0 ? 0.5 : 1 }}
                        >
                          <Play size={15} fill="#fff" /> เล่นทั้งหมด ({waypoints.length} ท่า)
                        </button>
                      ) : (
                        <button
                          onClick={stopPlayback}
                          className="h-11 rounded-xl flex items-center justify-center gap-2 text-sm font-medium"
                          style={{ background: C.red, color: "#fff" }}
                        >
                          <StopCircle size={15} /> หยุดเล่น
                        </button>
                      )}

                      {playing && playIndex >= 0 && waypoints[playIndex] && (
                        <div className="rounded-lg px-3 py-2.5" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}>
                          <div className="flex items-center justify-between text-xs mb-1.5">
                            <span style={{ color: C.sub }}>กำลังเล่น</span>
                            <span style={{ color: C.text }} className="font-medium">
                              {playIndex + 1} / {waypoints.length} · {waypoints[playIndex].name}
                            </span>
                          </div>
                          <div className="h-1.5 rounded-full overflow-hidden" style={{ background: C.track }}>
                            <div
                              className="h-full rounded-full"
                              style={{ width: `${Math.round(playProgress * 100)}%`, background: C.accent, transition: "width 60ms linear" }}
                            />
                          </div>
                        </div>
                      )}
                    </div>
                  </Panel>
                </div>

                {/* right column: waypoint list */}
                <Panel className="flex-1 min-w-[380px]">
                  <div className="flex items-center justify-between px-5 pt-4 pb-2">
                    <div>
                      <div className="text-[15px] font-semibold" style={{ color: C.text }}>รายการท่าทางที่บันทึกไว้</div>
                      <div className="text-xs mt-0.5" style={{ color: C.sub }}>{waypoints.length} ท่าทาง — เรียงตามลำดับการเล่น</div>
                    </div>
                  </div>
                  <div className="px-5 pb-5">
                    {waypoints.length === 0 ? (
                      <div className="rounded-xl py-10 flex flex-col items-center gap-2 text-center" style={{ background: C.panelAlt, border: `1px dashed ${C.borderSoft}` }}>
                        <PlayCircle size={22} color={C.subDim} />
                        <div className="text-sm" style={{ color: C.sub }}>ยังไม่มีท่าทางที่บันทึกไว้</div>
                        <div className="text-xs" style={{ color: C.subDim }}>เพิ่มท่าแรกได้จากช่องด้านซ้าย</div>
                      </div>
                    ) : (
                      <div className="flex flex-col gap-2">
                        {waypoints.map((wp, i) => {
                          const isPlaying = playing && playIndex === i;
                          return (
                            <div
                              key={wp.id}
                              className="flex items-center gap-3 rounded-xl px-3.5 py-3"
                              style={{
                                background: isPlaying ? C.accentSoft : C.panelAlt,
                                border: `1px solid ${isPlaying ? C.accent : C.borderSoft}`,
                              }}
                            >
                              <div
                                className="w-7 h-7 shrink-0 rounded-full flex items-center justify-center text-xs font-semibold tabular-nums"
                                style={{ background: isPlaying ? C.accent : C.panel, color: isPlaying ? "#fff" : C.sub, border: `1px solid ${C.borderSoft}` }}
                              >
                                {i + 1}
                              </div>
                              <div className="flex-1 min-w-0">
                                <div className="text-sm font-medium truncate" style={{ color: C.text }}>{wp.name}</div>
                                <div className="text-[11px] font-mono truncate" style={{ color: C.subDim }}>
                                  J1 {wp.joints.j1.toFixed(0)}° · J2 {wp.joints.j2.toFixed(0)}° · J3 {wp.joints.j3.toFixed(0)}° · J4 {wp.joints.j4.toFixed(0)}° · J5 {wp.joints.j5.toFixed(0)}%
                                </div>
                              </div>
                              <div className="flex items-center gap-1 shrink-0">
                                <button title="ย้ายขึ้น" onClick={() => moveWaypoint(wp.id, -1)} disabled={i === 0}
                                  className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: i === 0 ? C.subDim : C.sub, opacity: i === 0 ? 0.4 : 1 }}>
                                  <ArrowUp size={14} />
                                </button>
                                <button title="ย้ายลง" onClick={() => moveWaypoint(wp.id, 1)} disabled={i === waypoints.length - 1}
                                  className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: i === waypoints.length - 1 ? C.subDim : C.sub, opacity: i === waypoints.length - 1 ? 0.4 : 1 }}>
                                  <ArrowDown size={14} />
                                </button>
                                <button title="ไปยังตำแหน่งนี้" onClick={() => jumpToWaypoint(wp)} disabled={estopped || playing}
                                  className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: C.sub, opacity: estopped || playing ? 0.4 : 1 }}>
                                  <Target size={14} />
                                </button>
                                <button title="เล่นจากท่านี้" onClick={() => playFrom(wp.id)} disabled={estopped || playing}
                                  className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: C.accent, opacity: estopped || playing ? 0.4 : 1 }}>
                                  <Play size={14} />
                                </button>
                                <button title="ลบ" onClick={() => deleteWaypoint(wp.id)} disabled={playing}
                                  className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: C.red, opacity: playing ? 0.4 : 1 }}>
                                  <Trash2 size={14} />
                                </button>
                              </div>
                            </div>
                          );
                        })}
                      </div>
                    )}
                  </div>
                </Panel>
              </div>
            </>
          </div>

          <div style={{ display: activeTab === "status" ? "block" : "none" }}>
            <>
              <div className="mb-4">
                <h1 className="text-xl font-semibold" style={{ color: C.text }}>System Status</h1>
                <p className="text-xs mt-0.5" style={{ color: C.sub }}>สถานะปัจจุบันของทุกแกนและระบบ</p>
              </div>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <Panel>
                  <PanelHeader title="มุมข้อต่อปัจจุบัน" />
                  <div className="px-5 pb-5 flex flex-col gap-1">
                    {JOINTS.map((j) => (
                      <StatusRow key={j.key} label={`${j.label} (${j.sub})`} value={`${joints[j.key].toFixed(j.unit === "%" ? 0 : 1)}${j.unit === "%" ? "%" : "°"}`} />
                    ))}
                  </div>
                </Panel>
                <Panel>
                  <PanelHeader title="ตำแหน่งและการวางแนวปลายแขน" />
                  <div className="px-5 pb-5 flex flex-col gap-1">
                    <StatusRow label="X" value={`${pose.x.toFixed(1)} mm`} />
                    <StatusRow label="Y" value={`${pose.y.toFixed(1)} mm`} />
                    <StatusRow label="Z" value={`${pose.z.toFixed(1)} mm`} />
                    <StatusRow label="Roll" value={`${pose.roll.toFixed(1)}°`} />
                    <StatusRow label="Pitch" value={`${pose.pitch.toFixed(1)}°`} />
                    <StatusRow label="Yaw" value={`${pose.yaw.toFixed(1)}°`} />
                  </div>
                </Panel>
                <Panel>
                  <PanelHeader title="ฮาร์ดแวร์ / การเชื่อมต่อ" />
                  <div className="px-5 pb-5 flex flex-col gap-1">
                    <StatusRow label="พอร์ต" value={selectedPort} />
                    <StatusRow label="Baud rate" value="115200 bps" />
                    <StatusRow label="แรงดันไฟเลี้ยง" value={`${telemetry.voltage.toFixed(1)} V`} />
                    <StatusRow label="อุณหภูมิระบบ" value={`${telemetry.temp.toFixed(1)} °C`} />
                    <StatusRow label="FPS" value={telemetry.fps} />
                  </div>
                </Panel>
                <Panel>
                  <PanelHeader title="สถานะการทำงาน" />
                  <div className="px-5 pb-5 flex flex-col gap-1">
                    <StatusRow label="โหมดการทำงาน" value={{ normal: "โหมดปกติ", smooth: "โหมดนุ่มนวล", fast: "โหมดเร็ว", precise: "โหมดละเอียด" }[mode]} />
                    <StatusRow label="ความเร็ว" value={`${speed}%`} />
                    <StatusRow label="Emergency stop" value={estopped ? "ทำงาน" : "ปกติ"} valueColor={estopped ? C.red : C.green} />
                  </div>
                </Panel>
              </div>
            </>
          </div>

          <div className="max-w-2xl" style={{ display: activeTab === "about" ? "block" : "none" }}>
            <div className="mb-4">
                <h1 className="text-xl font-semibold" style={{ color: C.text }}>About Program</h1>
                <p className="text-xs mt-0.5" style={{ color: C.sub }}>เกี่ยวกับโปรแกรม</p>
              </div>
              <Panel>
                <div className="px-6 py-6 flex flex-col gap-4">
                  <div>
                    <div className="text-[11px] uppercase tracking-wide mb-1" style={{ color: C.subDim }}>ชื่อโครงการ</div>
                    <div className="text-lg font-semibold" style={{ color: C.text }}>แขนกลต้นทุนต่ำ</div>
                    <div className="text-sm" style={{ color: C.sub }}>Low-Cost Robot Arm (5-DOF)</div>
                  </div>
                  <div>
                    <div className="text-[11px] uppercase tracking-wide mb-1.5" style={{ color: C.subDim }}>คำอธิบาย</div>
                    <p className="text-sm leading-relaxed" style={{ color: C.sub }}>
                      โปรแกรมควบคุมแขนกล 5 แกนต้นทุนต่ำ รองรับการควบคุมแบบเรียลไทม์ผ่าน Serial (USB)
                      คำนวณตำแหน่งปลายแขนด้วย Forward Kinematics ขับเคลื่อนด้วยมอเตอร์ N20 AB Encoder
                      พร้อมระบบบันทึกและเล่นซ้ำท่าทาง (Record &amp; Playback) และปลายจับแบบ Symmetric Gripper
                      รูปทรงแขนกลทั้งหมดแสดงผลจากโมเดล 3D ที่สร้างใน Blender (.glb) โดยตรง
                    </p>
                  </div>
                  <div className="grid grid-cols-2 gap-3 pt-1">
                    {[
                      ["จำนวนแกน", "5 แกน (J1 หมุน + J2-J4 ขึ้น-ลง + J5 Gripper)"],
                      ["การสื่อสาร", "Serial USB (115200 bps)"],
                      ["การคำนวณตำแหน่ง", "Forward Kinematics (DH Parameters)"],
                      ["ระบบขับเคลื่อน", "N20 AB Encoder Motor"],
                    ].map(([k, v]) => (
                      <div key={k} className="rounded-xl px-4 py-3" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}>
                        <div className="text-[11px] mb-1" style={{ color: C.subDim }}>{k}</div>
                        <div className="text-sm font-medium" style={{ color: C.text }}>{v}</div>
                      </div>
                    ))}
                  </div>
                </div>
              </Panel>
            </div>
          </div>
        </div>

      {/* ---------------- Footer ---------------- */}
      <div
        className="flex items-center justify-end gap-5 px-5 py-2.5 shrink-0 text-xs"
        style={{ background: C.panel, borderTop: `1px solid ${C.border}`, color: C.subDim }}
      >
        <span className="flex items-center gap-1.5"><Clock size={13} /> {timeStr}</span>
        <span className="flex items-center gap-1.5"><CalendarDays size={13} /> {dateStr}</span>
      </div>

      {/* persistent "now playing" bar — visible on any tab while a sequence runs */}
      {playing && playIndex >= 0 && waypoints[playIndex] && (
        <div
          className="fixed bottom-6 left-1/2 -translate-x-1/2 w-[min(92vw,420px)] rounded-2xl px-4 py-3 shadow-2xl flex items-center gap-3"
          style={{ background: C.panel, border: `1px solid ${C.accent}` }}
        >
          <div className="w-8 h-8 shrink-0 rounded-lg flex items-center justify-center" style={{ background: C.accentSoft }}>
            <PlayCircle size={16} color={C.accent} />
          </div>
          <div className="flex-1 min-w-0">
            <div className="flex items-center justify-between text-xs mb-1">
              <span className="truncate font-medium" style={{ color: C.text }}>
                กำลังเล่น: {waypoints[playIndex].name}
              </span>
              <span className="shrink-0 ml-2" style={{ color: C.subDim }}>{playIndex + 1}/{waypoints.length}</span>
            </div>
            <div className="h-1.5 rounded-full overflow-hidden" style={{ background: C.track }}>
              <div
                className="h-full rounded-full"
                style={{ width: `${Math.round(playProgress * 100)}%`, background: C.accent, transition: "width 60ms linear" }}
              />
            </div>
          </div>
          <button
            onClick={stopPlayback}
            className="w-8 h-8 shrink-0 rounded-lg flex items-center justify-center"
            style={{ background: C.redSoft, color: C.red }}
            title="หยุดเล่น"
          >
            <StopCircle size={16} />
          </button>
        </div>
      )}

      {/* toast */}
      {toast && (
        <div
          className={`fixed left-1/2 -translate-x-1/2 px-4 py-2.5 rounded-xl text-sm font-medium shadow-lg ${playing ? "bottom-24" : "bottom-6"}`}
          style={{ background: C.accent, color: "#fff" }}
        >
          {toast}
        </div>
      )}
    </div>
  );
}
