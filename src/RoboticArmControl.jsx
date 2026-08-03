import React, { useEffect, useRef, useState, useCallback } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import robotModel from "./assets/arm_robotics.glb";
import { Bot, Wifi, ChevronDown } from "lucide-react";

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

  const viewerRef = useRef(null);
  const { modelReady, modelError } = useArmScene(viewerRef, HOME, false);

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
        </div>
      </div>
    </div>
  );
}
