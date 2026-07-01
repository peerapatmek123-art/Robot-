import React, { useEffect, useRef, useState, useCallback } from "react";
import * as THREE from "three";
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
  Minus,
  Plus,
  Clock,
  CalendarDays,
  Minus as WinMin,
  Square as WinMax,
  X as WinClose,
  ChevronDown,
  Wifi,
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

const JOINTS = [
  { key: "j1", label: "J1", sub: "ฐาน" },
  { key: "j2", label: "J2", sub: "ไหล่" },
  { key: "j3", label: "J3", sub: "ข้อศอก" },
  { key: "j4", label: "J4", sub: "ข้อมือ 1" },
  { key: "j5", label: "J5", sub: "ข้อมือ 2" },
  { key: "j6", label: "J6", sub: "ข้อมือ 3 / จับ" },
];

const HOME = { j1: 0, j2: 10, j3: 15, j4: 30, j5: 20, j6: 0 };

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
function useArmScene(containerRef, joints, wireframe) {
  const sceneRef = useRef(null);

  // init once
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return undefined;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(C.panelAlt);

    const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    container.appendChild(renderer.domElement);
    renderer.domElement.style.display = "block";
    renderer.domElement.style.cursor = "grab";

    // lights
    scene.add(new THREE.AmbientLight(0xffffff, 0.65));
    const key = new THREE.DirectionalLight(0xffffff, 0.9);
    key.position.set(3, 5, 4);
    scene.add(key);
    const rim = new THREE.DirectionalLight(0x3b6cf6, 0.35);
    rim.position.set(-4, 2, -3);
    scene.add(rim);

    // floor grid
    const grid = new THREE.GridHelper(8, 24, 0x27305a, 0x161c36);
    grid.position.y = 0;
    scene.add(grid);
    const axes = new THREE.AxesHelper(0.55);
    scene.add(axes);

    // ---- build arm hierarchy -------------------------------------------------
    const armColor = 0xe7ebf5;
    const jointColor = 0x1c2340;
    const accentColor = 0x3b6cf6;

    function link(length, r0 = 0.11, r1 = 0.095, color = armColor) {
      const g = new THREE.Group();
      const geo = new THREE.CylinderGeometry(r1, r0, length, 20);
      const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.35, roughness: 0.45 });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.y = length / 2;
      g.add(mesh);
      g.userData.mesh = mesh;
      return g;
    }
    function jointMesh(radius = 0.155, color = jointColor) {
      const geo = new THREE.SphereGeometry(radius, 20, 20);
      const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.6, roughness: 0.25 });
      const m = new THREE.Mesh(geo, mat);
      return m;
    }

    const LEN = { base: 0.42, l1: 1.15, l2: 1.0, l3: 0.42, l4: 0.36, grip: 0.28 };

    const root = new THREE.Group();
    scene.add(root);

    // base (turret) — rotates on J1
    const baseGroup = new THREE.Group();
    root.add(baseGroup);
    const baseMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.42, 0.46, LEN.base, 28),
      new THREE.MeshStandardMaterial({ color: 0xf4f6fb, metalness: 0.4, roughness: 0.4 })
    );
    baseMesh.position.y = LEN.base / 2;
    baseGroup.add(baseMesh);

    // shoulder — J2
    const shoulder = new THREE.Group();
    shoulder.position.y = LEN.base;
    baseGroup.add(shoulder);
    shoulder.add(jointMesh(0.19, accentColor));
    const upperArm = link(LEN.l1);
    shoulder.add(upperArm);

    // elbow — J3
    const elbow = new THREE.Group();
    elbow.position.y = LEN.l1;
    shoulder.add(elbow);
    elbow.add(jointMesh(0.16));
    const foreArm = link(LEN.l2, 0.095, 0.08);
    elbow.add(foreArm);

    // wrist roll — J4
    const wristRoll = new THREE.Group();
    wristRoll.position.y = LEN.l2;
    elbow.add(wristRoll);
    wristRoll.add(jointMesh(0.125, accentColor));
    const wristLink1 = link(LEN.l3, 0.075, 0.065);
    wristRoll.add(wristLink1);

    // wrist pitch — J5
    const wristPitch = new THREE.Group();
    wristPitch.position.y = LEN.l3;
    wristRoll.add(wristPitch);
    wristPitch.add(jointMesh(0.1));
    const wristLink2 = link(LEN.l4, 0.06, 0.05);
    wristPitch.add(wristLink2);

    // gripper — J6
    const gripper = new THREE.Group();
    gripper.position.y = LEN.l4;
    wristPitch.add(gripper);
    gripper.add(jointMesh(0.08, accentColor));
    const gripperBody = new THREE.Mesh(
      new THREE.CylinderGeometry(0.05, 0.06, LEN.grip, 12),
      new THREE.MeshStandardMaterial({ color: 0xf4f6fb, metalness: 0.4, roughness: 0.4 })
    );
    gripperBody.position.y = LEN.grip / 2;
    gripper.add(gripperBody);
    const fingerGeo = new THREE.BoxGeometry(0.03, 0.14, 0.05);
    const fingerMat = new THREE.MeshStandardMaterial({ color: 0x1c2340, metalness: 0.5, roughness: 0.3 });
    const fingerL = new THREE.Mesh(fingerGeo, fingerMat);
    const fingerR = new THREE.Mesh(fingerGeo, fingerMat);
    fingerL.position.set(-0.06, LEN.grip + 0.07, 0);
    fingerR.position.set(0.06, LEN.grip + 0.07, 0);
    gripper.add(fingerL, fingerR);

    const endEffector = new THREE.Object3D();
    endEffector.position.y = LEN.grip + 0.14;
    gripper.add(endEffector);

    // simple orbit controller state (no OrbitControls in this three build)
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

    sceneRef.current = {
      scene, camera, renderer, controls,
      baseGroup, shoulder, elbow, wristRoll, wristPitch, gripper, endEffector,
      allMeshes: [baseMesh, upperArm.userData.mesh, foreArm.userData.mesh, wristLink1.userData.mesh, wristLink2.userData.mesh, gripperBody, fingerL, fingerR],
    };

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
    if (!s) return;
    const d = THREE.MathUtils.degToRad;
    s.baseGroup.rotation.y = d(joints.j1);
    s.shoulder.rotation.z = d(joints.j2);
    s.elbow.rotation.z = d(joints.j3);
    s.wristRoll.rotation.y = d(joints.j4);
    s.wristPitch.rotation.z = d(joints.j5);
    s.gripper.rotation.y = d(joints.j6);
    s.baseGroup.updateMatrixWorld(true);

    const scale = 300; // model units -> mm, tuned to roughly match a ~700mm reach
    const pos = new THREE.Vector3();
    s.endEffector.getWorldPosition(pos);
    const quat = new THREE.Quaternion();
    s.endEffector.getWorldQuaternion(quat);
    const euler = new THREE.Euler().setFromQuaternion(quat, "XYZ");

    setPose({
      x: pos.x * scale,
      y: pos.z * scale,
      z: pos.y * scale,
      roll: THREE.MathUtils.radToDeg(euler.x),
      pitch: THREE.MathUtils.radToDeg(euler.z),
      yaw: THREE.MathUtils.radToDeg(euler.y),
    });
  }, [joints]);

  useEffect(() => {
    const s = sceneRef.current;
    if (!s) return;
    s.allMeshes.forEach((m) => { m.material.wireframe = wireframe; });
  }, [wireframe]);

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

  return { pose, resetView, zoom, setPanMode };
}

// ---------------------------------------------------------------------------
// Joint slider block
// ---------------------------------------------------------------------------
function JointControl({ label, sub, value, onChange, disabled }) {
  const step = 1;
  const clamp = (v) => Math.max(-180, Math.min(180, v));
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
          {value.toFixed(1)}
        </span>
        <span className="text-xs" style={{ color: C.subDim }}>deg</span>
      </div>
      <input
        type="range"
        min={-180}
        max={180}
        step={0.5}
        value={value}
        disabled={disabled}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="w-full accent-blue-500"
        style={{ accentColor: C.accent }}
      />
      <div className="flex items-center justify-between text-[10px] mt-1" style={{ color: C.subDim }}>
        <span>-180°</span>
        <span>180°</span>
      </div>
      <div className="flex items-center gap-2 mt-3">
        <button
          disabled={disabled}
          onClick={() => onChange(clamp(value - step))}
          className="flex-1 h-7 rounded-md flex items-center justify-center transition-colors"
          style={{ background: C.panel, border: `1px solid ${C.borderSoft}`, color: C.sub }}
        >
          <Minus size={13} />
        </button>
        <div
          className="flex-1 h-7 rounded-md flex items-center justify-center text-[11px]"
          style={{ background: C.panel, border: `1px solid ${C.borderSoft}`, color: C.subDim }}
        >
          1°
        </div>
        <button
          disabled={disabled}
          onClick={() => onChange(clamp(value + step))}
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
  const [connected, setConnected] = useState(true);

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

  const toggleConnection = () => {
    if (window.electronAPI?.connectPort) {
      if (connected) {
        window.electronAPI.disconnectPort();
        setConnected(false);
      } else {
        window.electronAPI.connectPort(selectedPort);
        setConnected(true);
      }
    } else {
      setConnected((v) => !v);
    }
  };
  const [now, setNow] = useState(new Date());
  const [telemetry, setTelemetry] = useState({ voltage: 12.0, temp: 36.2, fps: 60 });

  const containerRef = useRef(null);
  const { pose, resetView, zoom, setPanMode } = useArmScene(containerRef, joints, wireframe);

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

  const handleSave = () => showToast("บันทึกตำแหน่งปัจจุบันแล้ว");
  const handleHome = () => {
    if (estopped) return;
    setJoints(HOME);
    showToast("กลับสู่ตำแหน่งเริ่มต้นแล้ว");
  };
  const handleStop = () => {
    setEstopped((v) => !v);
    showToast(estopped ? "กลับมาทำงานตามปกติ" : "หยุดการทำงานฉุกเฉิน");
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
            6-DOF Robotic Arm Control
          </span>
        </div>

        <div className="flex items-center gap-3">
          <button
            onClick={toggleConnection}
            className="flex items-center gap-2 px-3 py-1.5 rounded-full text-xs"
            style={{ background: !connected ? C.redSoft : estopped ? C.redSoft : "rgba(34,197,94,0.12)" }}
          >
            <span
              className="w-1.5 h-1.5 rounded-full"
              style={{ background: !connected ? C.red : estopped ? C.red : C.green }}
            />
            <span style={{ color: !connected ? C.red : estopped ? C.red : C.green }}>
              {!connected ? "ไม่ได้เชื่อมต่อ" : estopped ? "หยุดฉุกเฉิน" : "เชื่อมต่อแล้ว"}
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

          <div className="flex items-center gap-1.5 pl-2" style={{ borderLeft: `1px solid ${C.border}` }}>
            <div className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: C.subDim }}>
              <WinMin size={14} />
            </div>
            <div className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: C.subDim }}>
              <WinMax size={13} />
            </div>
            <div className="w-7 h-7 rounded-md flex items-center justify-center" style={{ color: C.subDim }}>
              <WinClose size={14} />
            </div>
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
          {activeTab === "manual" && (
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
                  <div className="flex-1 mx-3 mb-3 rounded-xl overflow-hidden" ref={containerRef} style={{ minHeight: 300 }} />
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
                          value={joints[j.key]}
                          disabled={estopped}
                          onChange={(v) => setJoint(j.key, v)}
                        />
                      ))}
                    </div>

                    <div className="flex flex-wrap gap-3 mt-4">
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
                      <span className="flex items-center gap-1.5 text-sm font-medium" style={{ color: estopped ? C.red : C.green }}>
                        <span className="w-1.5 h-1.5 rounded-full" style={{ background: estopped ? C.red : C.green }} />
                        {estopped ? "หยุดฉุกเฉิน" : "เชื่อมต่อแล้ว"}
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
          )}

          {activeTab === "record" && (
            <div className="h-full flex items-center justify-center">
              <div className="text-center max-w-sm">
                <div className="w-14 h-14 mx-auto mb-4 rounded-2xl flex items-center justify-center" style={{ background: C.accentSoft }}>
                  <PlayCircle size={26} color={C.accent} />
                </div>
                <h2 className="text-lg font-semibold mb-1.5" style={{ color: C.text }}>Record &amp; Playback</h2>
                <p className="text-sm" style={{ color: C.sub }}>
                  โหมดบันทึกและเล่นท่าทางอัตโนมัติอยู่ระหว่างการพัฒนา — บันทึกตำแหน่งจากแท็บ Manual Control ไว้ก่อนได้เลยครับ
                </p>
              </div>
            </div>
          )}

          {activeTab === "status" && (
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
                      <StatusRow key={j.key} label={`${j.label} (${j.sub})`} value={`${joints[j.key].toFixed(1)}°`} />
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
          )}

          {activeTab === "about" && (
            <div className="max-w-2xl">
              <div className="mb-4">
                <h1 className="text-xl font-semibold" style={{ color: C.text }}>About Program</h1>
                <p className="text-xs mt-0.5" style={{ color: C.sub }}>เกี่ยวกับโปรแกรม</p>
              </div>
              <Panel>
                <div className="px-6 py-6 flex flex-col gap-4">
                  <div>
                    <div className="text-[11px] uppercase tracking-wide mb-1" style={{ color: C.subDim }}>ชื่อโครงการ</div>
                    <div className="text-lg font-semibold" style={{ color: C.text }}>แขนกลต้นทุนต่ำ</div>
                    <div className="text-sm" style={{ color: C.sub }}>Low-Cost RobotArm</div>
                  </div>
                  <div>
                    <div className="text-[11px] uppercase tracking-wide mb-1.5" style={{ color: C.subDim }}>คำอธิบาย</div>
                    <p className="text-sm leading-relaxed" style={{ color: C.sub }}>
                      โปรแกรมควบคุมแขนกล 6 แกนต้นทุนต่ำ รองรับการควบคุมแบบเรียลไทม์ผ่าน WebSocket
                      คำนวณตำแหน่งปลายแขนด้วย Forward Kinematics และขับเคลื่อนด้วย DC Motor
                      ร่วมกับระบบเฟือง Cycloidal Drive เพื่อให้ผู้เรียนทุกระดับเข้าถึงเทคโนโลยีหุ่นยนต์ได้จริง
                    </p>
                  </div>
                  <div className="grid grid-cols-2 gap-3 pt-1">
                    {[
                      ["จำนวนแกน", "6 แกน (2 หมุน + 4 ขยับ)"],
                      ["การสื่อสาร", "WebSocket แบบเรียลไทม์"],
                      ["การคำนวณตำแหน่ง", "Forward Kinematics"],
                      ["ระบบขับเคลื่อน", "DC Motor + Cycloidal Drive"],
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
          )}
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

      {/* toast */}
      {toast && (
        <div
          className="fixed bottom-6 left-1/2 -translate-x-1/2 px-4 py-2.5 rounded-xl text-sm font-medium shadow-lg"
          style={{ background: C.accent, color: "#fff" }}
        >
          {toast}
        </div>
      )}
    </div>
  );
}
