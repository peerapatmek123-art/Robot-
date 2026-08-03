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

const HOME = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0 };

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
function useArmScene(containerRef, joints, wireframe) {
  const sceneRef = useRef(null);
  const [modelReady, setModelReady] = useState(false);
  const [modelError, setModelError] = useState(false);

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

      s.endEffector = s.gripperGroup;

      // ---- Gizmo ลูกศรที่ปลายแขน (ปลายจับ) แสดงแกน X/Y/Z ของ tool ----
      // แดง = X, เขียว = Y, น้ำเงิน = Z (ตามธรรมเนียม CIRA Core / kinematic tool)
      const gizmoLen = 0.42;
      const gizmoGroup = new THREE.Group();
      gizmoGroup.name = "EndEffectorGizmo";
      const axisX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), gizmoLen, 0xef4444, gizmoLen * 0.28, gizmoLen * 0.14);
      const axisY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), gizmoLen, 0x22c55e, gizmoLen * 0.28, gizmoLen * 0.14);
      const axisZ = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), gizmoLen, 0x3b6cf6, gizmoLen * 0.28, gizmoLen * 0.14);
      [axisX, axisY, axisZ].forEach((a) => {
        a.line.material.depthTest = false;
        a.cone.material.depthTest = false;
        a.renderOrder = 999;
        gizmoGroup.add(a);
      });
      s.endEffector.add(gizmoGroup);
      s.gizmo = gizmoGroup;

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

  const viewerRef = useRef(null);
  const { modelReady, modelError } = useArmScene(viewerRef, joints, false);

  const handleTargetChange = (key, min, max, raw) => {
    const v = raw === "" ? "" : Math.max(min, Math.min(max, parseFloat(raw)));
    setTargets((t) => ({ ...t, [key]: raw === "" ? "" : v }));
  };

  const handleMove = useCallback(() => {
    if (moving) return;
    const from = { ...jointsRef.current };
    const to = { ...from };
    JOINTS.forEach((j) => {
      const v = targets[j.key];
      to[j.key] = v === "" || v === undefined || Number.isNaN(v) ? from[j.key] : v;
    });

    const duration = motionType === "LIN" ? 1400 : motionType === "CIRC" ? 1800 : 1100;
    setMoving(true);
    const start = performance.now();

    function step(now) {
      const t = Math.min(1, (now - start) / duration);
      let e;
      if (motionType === "LIN") {
        e = t; // ความเร็วคงที่ — เส้นทางแบบ Linear
      } else if (motionType === "CIRC") {
        // แทรกส่วนโค้งผ่านจุดกึ่งกลาง (via point) จำลองเส้นทางแบบ Circular
        e = easeInOutCubic(t);
      } else {
        e = easeInOutCubic(t); // PTP — แต่ละแกนเร่ง/ชะลอพร้อมกัน
      }

      const next = {};
      JOINTS.forEach((j) => {
        let val = from[j.key] + (to[j.key] - from[j.key]) * e;
        if (motionType === "CIRC") {
          // เพิ่ม bulge เล็กน้อยที่ J2/J3 ระหว่างทางเพื่อให้เห็นส่วนโค้งของ Circular move
          if (j.key === "j2" || j.key === "j3") {
            val += Math.sin(t * Math.PI) * 8 * (j.key === "j2" ? 1 : -1);
          }
        }
        next[j.key] = val;
      });
      setJoints(next);

      if (t < 1) {
        animRef.current = requestAnimationFrame(step);
      } else {
        setJoints(to);
        setMoving(false);
      }
    }
    animRef.current = requestAnimationFrame(step);
  }, [moving, motionType, targets]);

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

          {/* ---------------- Kinematic Move panel (PTP / LIN / CIRC) ---------------- */}
          <div
            className="absolute top-4 right-4 w-64 rounded-2xl overflow-hidden shadow-2xl"
            style={{ background: "rgba(16,21,42,0.92)", border: `1px solid ${C.border}`, backdropFilter: "blur(6px)" }}
          >
            <div className="flex items-center gap-2 px-4 pt-3.5 pb-2.5" style={{ borderBottom: `1px solid ${C.borderSoft}` }}>
              <Navigation size={14} color={C.accent} />
              <span className="text-xs font-semibold" style={{ color: C.text }}>Kinematic Move</span>
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

            {/* Target joint inputs */}
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

            {/* Move button */}
            <div className="px-4 py-3.5">
              <button
                onClick={handleMove}
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
                {moving ? `กำลังเคลื่อนที่ (${motionType})...` : `Move (${motionType})`}
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
