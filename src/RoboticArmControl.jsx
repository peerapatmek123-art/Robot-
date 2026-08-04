import React, { useEffect, useRef, useState, useCallback } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import robotModel from "./assets/arm_robotics.glb";
import { Bot, Wifi, ChevronDown, Home as HomeIcon, Move, Save, Trash2, Play, Activity, RotateCcw } from "lucide-react";

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
  accentSoft: "rgba(59,108,246,0.12)",
  green: "#22c55e",
  red: "#ef4444",
};

// ---------------------------------------------------------------------------
// Robot arm link lengths (เมตร) — ใช้แค่ J1 (ฐาน) / J2 (ไหล่) / J3 (ข้อศอก)
// ยังไม่คำนวณ J4/J5 ในเวอร์ชันนี้ (ตัดออกตามที่ขอ ให้เหลือแค่ลากแขนแบบง่ายก่อน)
// ---------------------------------------------------------------------------
const L1 = 0.10; // ความสูงไหล่จากพื้น (m)
const L2 = 0.105; // แขนบน ไหล่-ข้อศอก (m)
const L3 = 0.096; // แขนล่าง ข้อศอก-ข้อมือ (m)

const HOME = { j1: 0, j2: 0, j3: 90, j4: 90, j5: 0 };

/**
 * solveIK3 — Inverse Kinematics แบบง่าย (2-link planar + ฐานหมุน) หา J1/J2/J3
 * จากตำแหน่งเป้าหมาย (x,y,z) ของ "ข้อมือ" (ปลาย L3, จุดหมุน J4) โดยตรง
 * ไม่หักระยะปลายมือคีบ (L4) และไม่คำนวณ J4 ให้อยู่แนวระดับ — เก็บ J4/J5 ไว้ตามเดิม
 */
function solveIK3(x, y, z) {
  const j1 = THREE.MathUtils.radToDeg(Math.atan2(x, z));
  const r = Math.sqrt(x * x + z * z);
  const dy = y - L1;
  const dist = Math.sqrt(r * r + dy * dy);

  if (dist > L2 + L3 - 0.001 || dist < Math.abs(L2 - L3) + 0.001) {
    return { ok: false, j1, j2: 0, j3: 0 };
  }

  const cosJ3 = (dist * dist - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  const j3Mag = Math.acos(THREE.MathUtils.clamp(cosJ3, -1, 1));
  const alpha = Math.atan2(dy, r);
  const sinBetaMag = THREE.MathUtils.clamp((L3 * Math.sin(j3Mag)) / dist, -1, 1);
  const betaMag = Math.asin(sinBetaMag);

  const j2 = THREE.MathUtils.radToDeg(alpha + betaMag);
  const j3 = THREE.MathUtils.radToDeg(j3Mag);

  return {
    ok: true,
    j1: THREE.MathUtils.clamp(j1, -180, 180),
    j2: THREE.MathUtils.clamp(j2, -90, 90),
    j3: THREE.MathUtils.clamp(j3, -135, 135),
  };
}

// ---------------------------------------------------------------------------
// 3D Arm Scene — โหลดโมเดล GLTF, กล้อง orbit, ลูกศรลาก XYZ ที่ข้อมือ
// ---------------------------------------------------------------------------
function useArmScene(containerRef, joints, onIkDrag, trailControlRef) {
  const sceneRef = useRef(null);
  const [modelReady, setModelReady] = useState(false);
  const [modelError, setModelError] = useState(false);
  const onIkDragRef = useRef(onIkDrag);
  useEffect(() => { onIkDragRef.current = onIkDrag; }, [onIkDrag]);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return undefined;

    const scene = new THREE.Scene();

    // พื้นหลังไล่สี (เข้มด้านบน อ่อนลงด้านล่างเล็กน้อย)
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
    scene.add(new THREE.Mesh(bgGeo, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide, fog: false })));

    const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);
    renderer.domElement.style.display = "block";
    renderer.domElement.style.cursor = "grab";

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

    const floor = new THREE.Mesh(new THREE.CircleGeometry(4.4, 48), new THREE.ShadowMaterial({ opacity: 0.38 }));
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.001;
    floor.receiveShadow = true;
    scene.add(floor);
    const grid = new THREE.GridHelper(8, 32, 0x2c3766, 0x161c36);
    grid.material.opacity = 0.55;
    grid.material.transparent = true;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(0.55));

    // ---- กล้อง orbit ----
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

    // ---- ลากลูกศร (raycast) ----
    const raycaster = new THREE.Raycaster();
    const pointerNDC = new THREE.Vector2();
    const AXIS_VECTORS = {
      x: new THREE.Vector3(1, 0, 0),
      y: new THREE.Vector3(0, 1, 0),
      z: new THREE.Vector3(0, 0, 1),
    };
    const handleDrag = { active: false, spec: null, plane: new THREE.Plane(), lineOrigin: new THREE.Vector3(), lineDir: new THREE.Vector3(), lastT: 0 };
    const _camForward = new THREE.Vector3();
    const _camRight = new THREE.Vector3();
    const _planeNormal = new THREE.Vector3();
    const _intersectPoint = new THREE.Vector3();
    const _dragLineDir = new THREE.Vector3();

    function pointerRay(e) {
      const rect = renderer.domElement.getBoundingClientRect();
      pointerNDC.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      pointerNDC.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointerNDC, camera);
      return raycaster.ray;
    }

    function pickHandle(e) {
      const s = sceneRef.current;
      if (!s || !s.ready || !s.pickables || !s.pickables.length) return null;
      const ray = pointerRay(e);
      raycaster.ray.copy(ray);
      const hits = raycaster.intersectObjects(s.pickables, false);
      return hits.length ? hits[0].object.userData : null;
    }

    function onPointerDown(e) {
      const spec = pickHandle(e);
      const s = sceneRef.current;
      if (spec && spec.axis && onIkDragRef.current && s && s.gizmo) {
        const gizmoPos = s.gizmo.position;
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
      if (handleDrag.active) {
        const ray = pointerRay(e);
        const hit = ray.intersectPlane(handleDrag.plane, _intersectPoint);
        if (hit) {
          const t = _intersectPoint.clone().sub(handleDrag.lineOrigin).dot(handleDrag.lineDir);
          const dt = t - handleDrag.lastT;
          handleDrag.lastT = t;
          onIkDragRef.current?.(handleDrag.spec.axis, dt);
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

    sceneRef.current = {
      baseGroup: null,
      shoulder: null,
      elbow: null,
      wrist: null,
      allMeshes: [],
      pickables: [],
      gizmo: null,
      modelScale: 1,
      ready: false,
    };

    // ---- Trail system — วาดเส้น 3D ตามการเคลื่อนที่ปลายมือ ----
    const MAX_TRAIL_POINTS = 4000;
    const trailPositions = new Float32Array(MAX_TRAIL_POINTS * 3);
    const trailColors = new Float32Array(MAX_TRAIL_POINTS * 3);
    let trailCount = 0;
    let trailRecording = false;
    let lastTrailPos = null;
    const MIN_DIST = 0.004; // ระยะขั้นต่ำระหว่างจุด (เมตร)

    // สร้าง geometry แบบ dynamic draw
    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPositions, 3));
    trailGeo.setAttribute("color", new THREE.BufferAttribute(trailColors, 3));
    trailGeo.setDrawRange(0, 0);
    const trailMat = new THREE.LineBasicMaterial({
      vertexColors: true,
      linewidth: 2,
      depthTest: true,
      transparent: true,
      opacity: 0.92,
    });
    const trailLine = new THREE.Line(trailGeo, trailMat);
    trailLine.frustumCulled = false;
    trailLine.renderOrder = 10;
    scene.add(trailLine);

    // ---- จุดวาย (waypoint dots) — sphere เล็กๆ แต่ละ segment ----
    const waypointDots = [];

    function addTrailPoint(pos, motionType) {
      if (trailCount >= MAX_TRAIL_POINTS) return;
      const i = trailCount * 3;
      trailPositions[i]     = pos.x;
      trailPositions[i + 1] = pos.y;
      trailPositions[i + 2] = pos.z;

      // สีตาม motion type
      const col = motionType === "LIN"
        ? new THREE.Color(0x22c55e)
        : motionType === "CIRC"
        ? new THREE.Color(0xf59e0b)
        : new THREE.Color(0x3b6cf6); // PTP default
      trailColors[i]     = col.r;
      trailColors[i + 1] = col.g;
      trailColors[i + 2] = col.b;

      trailCount++;
      trailGeo.attributes.position.needsUpdate = true;
      trailGeo.attributes.color.needsUpdate = true;
      trailGeo.setDrawRange(0, trailCount);
    }

    function clearTrail() {
      trailCount = 0;
      lastTrailPos = null;
      trailGeo.setDrawRange(0, 0);
      trailGeo.attributes.position.needsUpdate = true;
      // ลบ waypoint dots
      waypointDots.forEach((d) => scene.remove(d));
      waypointDots.length = 0;
    }

    function addWaypointDot(pos, motionType) {
      const col = motionType === "LIN" ? 0x22c55e : motionType === "CIRC" ? 0xf59e0b : 0x3b6cf6;
      const geo = new THREE.SphereGeometry(0.012, 8, 6);
      const mat = new THREE.MeshBasicMaterial({ color: col, depthTest: true, transparent: true, opacity: 0.9 });
      const dot = new THREE.Mesh(geo, mat);
      dot.position.copy(pos);
      dot.renderOrder = 11;
      scene.add(dot);
      waypointDots.push(dot);
    }

    // expose ผ่าน ref ที่ส่งมาจากภายนอก
    if (trailControlRef) {
      trailControlRef.current = {
        start: (motionType) => {
          trailRecording = true;
          // เพิ่มจุดเริ่มต้นแยก segment ถ้ามี trail อยู่แล้ว
          if (trailCount > 0 && lastTrailPos) {
            addWaypointDot(lastTrailPos, motionType);
          }
        },
        stop: () => {
          trailRecording = false;
          if (lastTrailPos) addWaypointDot(lastTrailPos, "PTP");
        },
        clear: clearTrail,
        getCount: () => trailCount,
        setMotionType: (mt) => { if (trailControlRef.current) trailControlRef.current._motionType = mt; },
        _motionType: "PTP",
      };
    }

    // ---- Minimap — renderer ขนาดเล็ก top-view มุมบนซ้าย ----
    const MINI_SIZE = 180;
    const miniRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    miniRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    miniRenderer.setSize(MINI_SIZE, MINI_SIZE);
    miniRenderer.domElement.style.cssText = `
      position:absolute; top:16px; left:16px;
      width:${MINI_SIZE}px; height:${MINI_SIZE}px;
      border-radius:14px;
      border:1px solid rgba(28,35,64,0.9);
      background:rgba(10,14,26,0.82);
      backdrop-filter:blur(8px);
      pointer-events:none;
      z-index:20;
      box-shadow:0 4px 20px rgba(0,0,0,0.5);
    `;
    container.appendChild(miniRenderer.domElement);

    const miniCamera = new THREE.OrthographicCamera(-0.35, 0.35, 0.35, -0.35, 0.01, 20);
    miniCamera.position.set(0, 8, 0);
    miniCamera.lookAt(0, 0, 0);

    // Mini-scene แสดงเฉพาะ grid + trail (ไม่โหลดโมเดลซ้ำ)
    const miniScene = new THREE.Scene();
    const miniGrid = new THREE.GridHelper(0.7, 14, 0x2c3766, 0x1c2340);
    miniGrid.material.opacity = 0.7;
    miniGrid.material.transparent = true;
    miniScene.add(miniGrid);
    miniScene.add(new THREE.AxesHelper(0.12));

    // Trail line copy ใน mini scene — share geometry เดียวกัน
    const miniTrailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.95,
    }));
    miniTrailLine.frustumCulled = false;
    miniScene.add(miniTrailLine);

    // Current position dot ใน minimap
    const curDotGeo = new THREE.CircleGeometry(0.012, 10);
    const curDotMat = new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false });
    const curDot = new THREE.Mesh(curDotGeo, curDotMat);
    curDot.rotation.x = -Math.PI / 2;
    miniScene.add(curDot);

    // Label "PATH" บน minimap (canvas texture)
    function makeLabelCanvas(text) {
      const c = document.createElement("canvas");
      c.width = 100; c.height = 24;
      const cx2 = c.getContext("2d");
      cx2.fillStyle = "rgba(0,0,0,0)";
      cx2.fillRect(0, 0, c.width, c.height);
      cx2.fillStyle = "#7480a3";
      cx2.font = "bold 11px monospace";
      cx2.fillText(text, 4, 16);
      return new THREE.CanvasTexture(c);
    }

    // ---- ใน tick() อัปเดต trail และ minimap ----
    // เก็บ ref ไว้ใช้ใน tick
    const trailState = { recording: false, motionType: "PTP" };
    if (trailControlRef) {
      const orig = trailControlRef.current;
      trailControlRef.current = {
        ...orig,
        start: (mt) => { trailState.recording = true; trailState.motionType = mt || "PTP"; if (trailCount > 0 && lastTrailPos) addWaypointDot(lastTrailPos, mt); },
        stop: () => { trailState.recording = false; if (lastTrailPos) addWaypointDot(lastTrailPos, "PTP"); },
        clear: clearTrail,
        getCount: () => trailCount,
      };
    }

    const GIZMO_REF_DISTANCE = controls.radius;
    const _wristWorld = new THREE.Vector3();
    const _fingerLWorld = new THREE.Vector3();
    const _fingerRWorld = new THREE.Vector3();
    const _fingerLBox = new THREE.Box3();
    const _fingerRBox = new THREE.Box3();
    let raf;
    function tick() {
      applyCamera();
      const s = sceneRef.current;
      if (s && s.ready && s.wrist && s.gizmo) {
        if (!handleDrag.active) {
          if (s.fingerL && s.fingerR) {
            _fingerLBox.setFromObject(s.fingerL);
            _fingerRBox.setFromObject(s.fingerR);
            _fingerLBox.getCenter(_fingerLWorld);
            _fingerRBox.getCenter(_fingerRWorld);
            _wristWorld.copy(_fingerLWorld).add(_fingerRWorld).multiplyScalar(0.5);
          } else {
            s.wrist.getWorldPosition(_wristWorld);
          }
          s.gizmo.position.copy(_wristWorld);

          // ---- Trail sampling ----
          if (trailState.recording) {
            if (!lastTrailPos || _wristWorld.distanceTo(lastTrailPos) >= MIN_DIST) {
              addTrailPoint(_wristWorld, trailState.motionType);
              lastTrailPos = _wristWorld.clone();
            }
          }

          // อัปเดต current position dot ใน minimap
          curDot.position.set(_wristWorld.x, 0.001, _wristWorld.z);
        }
        const camDist = camera.position.distanceTo(s.gizmo.position);
        s.gizmo.scale.setScalar(camDist / GIZMO_REF_DISTANCE);
      }
      renderer.render(scene, camera);

      // ---- Render minimap ----
      miniRenderer.render(miniScene, miniCamera);

      raf = requestAnimationFrame(tick);
    }
    tick();

    const loader = new GLTFLoader();
    loader.load(
      robotModel,
      (gltf) => buildFromGLTF(gltf),
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
      model.position.y -= box.min.y;
      s.modelScale = model.scale.x || 1;

      s.baseGroup = model.getObjectByName("Gear_for_Base");
      s.shoulder = model.getObjectByName("ArmJ2");
      s.elbow = model.getObjectByName("ArmJ3");
      s.wrist = model.getObjectByName("ArmGriper");
      s.gripperGroup = model.getObjectByName("FingerBase");
      s.fingerL = model.getObjectByName("Left_Fringer");
      s.fingerR = model.getObjectByName("Right_Finger");

      const missing = ["baseGroup", "shoulder", "elbow", "wrist", "gripperGroup", "fingerL", "fingerR"].filter((k) => !s[k]);
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

      // ---- ลูกศร XYZ (แดง/เขียว/น้ำเงิน) ลากที่ข้อมือ ไม่ผูกกับหมุนของข้อต่อ ----
      const gizmoLen = 0.24;
      const HIT_PADDING = 2.0;
      function makeArrow(dir, color, axis) {
        const shaftLen = gizmoLen * 1.3;
        const headLen = gizmoLen * 0.28;
        const shaftRadius = gizmoLen * 0.025;
        const headRadius = gizmoLen * 0.08;
        const mat = new THREE.MeshBasicMaterial({ color, depthTest: false });
        const shaft = new THREE.Mesh(new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLen, 8), mat);
        const headPos = new THREE.Mesh(new THREE.ConeGeometry(headRadius, headLen, 10), mat);
        headPos.position.y = shaftLen / 2 + headLen / 2;
        const headNeg = new THREE.Mesh(new THREE.ConeGeometry(headRadius, headLen, 10), mat);
        headNeg.position.y = -(shaftLen / 2 + headLen / 2);
        headNeg.rotation.x = Math.PI;
        const totalLen = shaftLen + headLen * 2;
        const hitMat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0, depthTest: false });
        const hitBox = new THREE.Mesh(new THREE.CylinderGeometry(shaftRadius * HIT_PADDING, shaftRadius * HIT_PADDING, totalLen, 10), hitMat);
        hitBox.renderOrder = 998;
        const grp = new THREE.Group();
        grp.add(shaft, headPos, headNeg, hitBox);
        grp.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
        grp.renderOrder = 999;
        const userData = { axis };
        shaft.userData = userData;
        headPos.userData = userData;
        headNeg.userData = userData;
        hitBox.userData = userData;
        return { grp, pickables: [shaft, headPos, headNeg, hitBox] };
      }

      const gizmoGroup = new THREE.Group();
      gizmoGroup.name = "WristGizmo";
      s.pickables = [];
      [
        makeArrow(new THREE.Vector3(1, 0, 0), 0xef4444, "x"),
        makeArrow(new THREE.Vector3(0, 1, 0), 0x22c55e, "y"),
        makeArrow(new THREE.Vector3(0, 0, 1), 0x3b6cf6, "z"),
      ].forEach(({ grp, pickables }) => {
        gizmoGroup.add(grp);
        s.pickables.push(...pickables);
      });
      scene.add(gizmoGroup);
      s.gizmo = gizmoGroup;

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
      miniRenderer.dispose();
      if (container.contains(renderer.domElement)) container.removeChild(renderer.domElement);
      if (container.contains(miniRenderer.domElement)) container.removeChild(miniRenderer.domElement);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // ---- อัปเดตมุมข้อต่อจริงบนโมเดลทุกครั้งที่ joints เปลี่ยน (J1/J2/J3 เท่านั้น) ----
  useEffect(() => {
    const s = sceneRef.current;
    if (!s || !s.ready) return;
    const d = THREE.MathUtils.degToRad;
    s.baseGroup.rotation.y = d(joints.j1);
    s.shoulder.rotation.x = d(-joints.j2);
    s.elbow.rotation.y = d(-joints.j3);
    // J4/J5 คงค่าตามที่โหลดมา (ยังไม่คำนวณในเวอร์ชันนี้)
    s.wrist.rotation.y = d(-joints.j4);
    s.baseGroup.updateMatrixWorld(true);
  }, [joints]);

  return { modelReady, modelError };
}
// ---------------------------------------------------------------------------
// Main component
// ---------------------------------------------------------------------------
export default function RoboticArmControl() {
  const [ports, setPorts] = useState(["COM3"]);
  const [selectedPort, setSelectedPort] = useState("COM3");
  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);

  // Trail recording controls
  const trailControlRef = useRef(null);
  const [trailActive, setTrailActive] = useState(false);
  const [trailCount, setTrailCount] = useState(0);

  const [joints, setJoints] = useState(HOME);
  const jointsRef = useRef(HOME);
  useEffect(() => { jointsRef.current = joints; }, [joints]);

  // ---- แผงควบคุม "Kinematic Move" ----
  const [motionType, setMotionType] = useState("PTP"); // "PTP" | "LIN" | "CIRC"
  const [jointInputs, setJointInputs] = useState({
    j1: String(HOME.j1),
    j2: String(HOME.j2),
    j3: String(HOME.j3),
    j4: String(HOME.j4),
    j5: String(HOME.j5),
  });

  // sync ช่องกรอกให้ตรงกับตำแหน่งจริงเสมอ เมื่อ joints เปลี่ยนจากแหล่งอื่น (ลาก IK / ปุ่ม Home)
  useEffect(() => {
    setJointInputs({
      j1: String(joints.j1.toFixed ? joints.j1.toFixed(2) : joints.j1),
      j2: String(joints.j2.toFixed ? joints.j2.toFixed(2) : joints.j2),
      j3: String(joints.j3.toFixed ? joints.j3.toFixed(2) : joints.j3),
      j4: String(joints.j4.toFixed ? joints.j4.toFixed(2) : joints.j4),
      j5: String(joints.j5.toFixed ? joints.j5.toFixed(2) : joints.j5),
    });
  }, [joints]);

  const handleJointInputChange = useCallback((key, value) => {
    // อนุญาตให้พิมพ์ค่าติดลบ/จุดทศนิยม/ช่องว่างระหว่างพิมพ์ได้อย่างอิสระ
    if (value === "" || value === "-" || /^-?\d*\.?\d*$/.test(value)) {
      setJointInputs((prev) => ({ ...prev, [key]: value }));
    }
  }, []);

  // ---- เคลื่อนที่จริงแบบค่อยๆ ขยับ (ไม่ใช่กระโดดไปทันที) ----
  const [isMoving, setIsMoving] = useState(false);
  const moveAnimRef = useRef(null);

  const animateToJoints = useCallback((target, duration = 1100) => {
    return new Promise((resolve) => {
      if (moveAnimRef.current) cancelAnimationFrame(moveAnimRef.current);
      const start = { ...jointsRef.current };
      const startTime = performance.now();
      const easeOutCubic = (t) => 1 - Math.pow(1 - t, 3);

      setIsMoving(true);
      function step(now) {
        const t = Math.min(1, (now - startTime) / duration);
        const e = easeOutCubic(t);
        const next = {
          j1: start.j1 + (target.j1 - start.j1) * e,
          j2: start.j2 + (target.j2 - start.j2) * e,
          j3: start.j3 + (target.j3 - start.j3) * e,
          j4: start.j4 + (target.j4 - start.j4) * e,
          j5: start.j5 + (target.j5 - start.j5) * e,
        };
        jointsRef.current = next;
        setJoints(next);
        if (t < 1) {
          moveAnimRef.current = requestAnimationFrame(step);
        } else {
          moveAnimRef.current = null;
          setIsMoving(false);
          resolve();
        }
      }
      moveAnimRef.current = requestAnimationFrame(step);
    });
  }, []);

  useEffect(() => () => {
    if (moveAnimRef.current) cancelAnimationFrame(moveAnimRef.current);
  }, []);

  const handleMovePTP = useCallback(() => {
    const parsed = {
      j1: parseFloat(jointInputs.j1),
      j2: parseFloat(jointInputs.j2),
      j3: parseFloat(jointInputs.j3),
      j5: parseFloat(jointInputs.j5),
    };
    const targetJoints = {
      j1: Number.isFinite(parsed.j1) ? parsed.j1 : jointsRef.current.j1,
      j2: Number.isFinite(parsed.j2) ? parsed.j2 : jointsRef.current.j2,
      j3: Number.isFinite(parsed.j3) ? parsed.j3 : jointsRef.current.j3,
      j4: 90,
      j5: Number.isFinite(parsed.j5) ? parsed.j5 : jointsRef.current.j5,
    };
    // บันทึก trail อัตโนมัติระหว่างการเคลื่อนที่
    if (trailControlRef.current) {
      trailControlRef.current.start(motionType);
      setTrailActive(true);
      animateToJoints(targetJoints).then(() => {
        trailControlRef.current?.stop();
        setTrailActive(false);
      });
    } else {
      animateToJoints(targetJoints);
    }
  }, [jointInputs, animateToJoints, motionType]);

  // ---- ลากลูกศร 3 แกน (X/Y/Z) ที่ปลายมือคีบ ควบคุมข้อต่อโดยตรง ----
  // แกน X: หมุนเฉพาะ J1 (ฐาน) ตามทิศที่ลาก — J2/J3/J4/J5 คงค่าเดิม
  // แกน Y: ลากขึ้น (delta บวก) → J2/J3 เคลื่อนที่เข้าหา 0 องศา
  //         ลากลง (delta ลบ) → J2/J3 หมุนไปตามทิศที่ลาก (เหมือนแกน Z)
  // แกน Z: J2/J3 หมุนออกจาก 0 องศา ไปทางทิศที่ลาก (ลากบวก → มุมเพิ่ม, ลากลบ → มุมลด)
  // J4 เป็นค่าคงตัวที่ 90 องศาเสมอ ไม่ถูกควบคุมโดยการลากแกนใดๆ
  const DEG_PER_UNIT = 400; // ความไว: องศาต่อระยะลาก 1 หน่วย (เมตร) — ปรับได้ตรงนี้

  // ---- บันทึกท่าทาง (Saved Poses) ----
  const [savedPoses, setSavedPoses] = useState([]); // [{ id, name, joints }]

  const handleSavePose = useCallback(() => {
    setSavedPoses((prev) => [
      ...prev,
      { id: Date.now(), name: `ท่าที่ ${prev.length + 1}`, joints: { ...jointsRef.current } },
    ]);
  }, []);

  const handleGoToPose = useCallback((pose) => {
    animateToJoints(pose.joints);
  }, [animateToJoints]);

  // ---- เล่นท่าทางทั้งหมดต่อเนื่องกัน — เริ่มจากตำแหน่งเริ่มต้น (HOME) เองโดยไม่ต้องกด Home ก่อน ----
  const [isPlayingAll, setIsPlayingAll] = useState(false);

  const handlePlayAll = useCallback(async () => {
    if (isPlayingAll || savedPoses.length === 0) return;
    setIsPlayingAll(true);
    await animateToJoints(HOME);
    for (const pose of savedPoses) {
      await animateToJoints(pose.joints);
    }
    setIsPlayingAll(false);
  }, [isPlayingAll, savedPoses, animateToJoints]);

  const handleDeletePose = useCallback((id) => {
    setSavedPoses((prev) => prev.filter((p) => p.id !== id));
  }, []);

  const handleIkDrag = useCallback((axis, delta) => {
    const cur = jointsRef.current;
    const clampDeg = (v, min, max) => Math.max(min, Math.min(max, v));
    const moveToward = (val, target, step) => {
      if (val > target) return Math.max(target, val - step);
      if (val < target) return Math.min(target, val + step);
      return target;
    };

    const next = { ...cur };
    if (axis === "x") {
      next.j1 = clampDeg(cur.j1 + delta * DEG_PER_UNIT, -180, 180);
    } else if (axis === "y") {
      if (delta > 0) {
        // ลากขึ้น — J2/J3 เข้าหา 0 องศา
        const step = delta * DEG_PER_UNIT;
        next.j2 = moveToward(cur.j2, 0, step);
        next.j3 = moveToward(cur.j3, 0, step);
      } else {
        // ลากลง — J2 ไปทางลบ, J3 ไปทางบวก
        const step = Math.abs(delta) * DEG_PER_UNIT;
        next.j2 = clampDeg(cur.j2 - step, -90, 90);
        next.j3 = clampDeg(cur.j3 + step, -135, 135);
      }
    } else if (axis === "z") {
      const step = delta * DEG_PER_UNIT;
      next.j2 = clampDeg(cur.j2 + step, -90, 90);
      next.j3 = clampDeg(cur.j3 + step, -135, 135);
    }

    jointsRef.current = next;
    setJoints(next);
  }, []);

  const viewerRef = useRef(null);
  const { modelReady, modelError } = useArmScene(viewerRef, joints, handleIkDrag, trailControlRef);

  // ---- ปุ่ม Home: เคลื่อนที่กลับตำแหน่งเริ่มต้นแบบค่อยๆ ขยับ ----
  const handleHome = useCallback(() => {
    animateToJoints(HOME);
  }, [animateToJoints]);

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
      if (!list.includes(selectedPort)) setSelectedPort(list[0]);
    } catch (err) {
      console.error(err);
    }
  }, [selectedPort]);

  useEffect(() => { refreshPorts(); }, [refreshPorts]);

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
      <style>{`@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }`}</style>
      {/* ---------------- Header ---------------- */}
      <div className="flex items-center justify-between px-5 py-3 shrink-0" style={{ background: C.panel, borderBottom: `1px solid ${C.border}` }}>
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2.5">
            <div className="w-8 h-8 rounded-lg flex items-center justify-center" style={{ background: C.accentSoft }}>
              <Bot size={18} color={C.accent} />
            </div>
            <span className="text-[15px] font-semibold" style={{ color: C.text }}>ANR Robot Studio</span>
          </div>

        </div>

        <div className="flex items-center gap-3">
          <button
            onClick={handleHome}
            disabled={!modelReady || isMoving || isPlayingAll}
            title="กลับตำแหน่งเริ่มต้น (Home)"
            className="flex items-center gap-1.5 px-3.5 py-1.5 rounded-full text-xs font-medium transition-colors"
            style={{
              background: C.panelAlt,
              color: !modelReady || isMoving || isPlayingAll ? C.subDim : C.text,
              border: `1px solid ${C.borderSoft}`,
              cursor: !modelReady || isMoving || isPlayingAll ? "default" : "pointer",
              opacity: !modelReady || isMoving || isPlayingAll ? 0.6 : 1,
            }}
          >
            <HomeIcon size={13} />
            Home
          </button>

          <div className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs relative" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text }}>
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
            <span className="w-1.5 h-1.5 rounded-full" style={{ background: connected ? C.green : "#fff" }} />
            {connecting ? "กำลังเชื่อมต่อ..." : connected ? "เชื่อมต่อแล้ว" : "Connect"}
          </button>
        </div>
      </div>

      {/* ---------------- 3D Robot Arm Viewer + Control Panel (floating overlay) ---------------- */}
      <div className="flex-1 min-h-0 p-4 relative">
        <div className="w-full h-full rounded-2xl overflow-hidden relative" style={{ background: C.panel, border: `1px solid ${C.border}` }}>
          <div ref={viewerRef} className="w-full h-full" />
          {!modelReady && !modelError && (
            <div className="absolute inset-0 flex items-center justify-center text-sm" style={{ color: C.sub }}>
              กำลังโหลดโมเดล 3D...
            </div>
          )}
          {modelError && (
            <div className="absolute inset-0 flex items-center justify-center text-sm" style={{ color: C.red }}>
              โหลดโมเดล 3D ไม่สำเร็จ
            </div>
          )}

          {/* Minimap label overlay — HTML on top of Three.js minimap canvas at top-left */}
          {modelReady && (
            <div
              className="absolute pointer-events-none"
              style={{ top: 16, left: 16, width: 180, height: 180, zIndex: 25 }}
            >
              {/* Title bar at bottom of minimap */}
              <div
                className="absolute bottom-0 left-0 right-0 flex items-center justify-between px-2.5 py-1.5 rounded-b-xl"
                style={{
                  background: "rgba(10,14,26,0.88)",
                  borderTop: `1px solid rgba(28,35,64,0.8)`,
                }}
              >
                <div className="flex items-center gap-1.5">
                  <Activity size={9} color={C.accent} />
                  <span style={{ fontSize: 9, fontFamily: "monospace", color: C.sub, fontWeight: 600, letterSpacing: "0.05em" }}>
                    PATH VIEW
                  </span>
                </div>
                {trailActive && (
                  <div className="flex items-center gap-1">
                    <span className="w-1.5 h-1.5 rounded-full" style={{ background: C.red, boxShadow: `0 0 4px ${C.red}`, animation: "pulse 1s infinite" }} />
                    <span style={{ fontSize: 8, color: C.red, fontFamily: "monospace" }}>REC</span>
                  </div>
                )}
              </div>
            </div>
          )}
        </div>

        {/* ---------------- Kinematic Move Panel (floating glass card) ---------------- */}
        <div
          className="absolute top-8 right-8 w-[300px] rounded-2xl p-4 flex flex-col gap-4"
          style={{
            background: "rgba(16,21,42,0.72)",
            border: `1px solid ${C.border}`,
            backdropFilter: "blur(14px)",
            WebkitBackdropFilter: "blur(14px)",
            boxShadow: "0 12px 32px rgba(0,0,0,0.45)",
          }}
        >
          {/* Header: title */}
          <div className="flex items-center gap-2">
            <Move size={15} color={C.accent} />
            <span className="text-[13px] font-semibold" style={{ color: C.text }}>Kinematic Move</span>
          </div>

          {/* Motion type */}
          <div>
            <div className="text-[10px] font-semibold tracking-wide mb-1.5" style={{ color: C.subDim }}>
              MOTION TYPE
            </div>
            <div className="flex gap-1.5">
              {["PTP", "LIN", "CIRC"].map((t) => (
                <button
                  key={t}
                  onClick={() => setMotionType(t)}
                  className="flex-1 py-1.5 rounded-lg text-xs font-medium transition-colors"
                  style={{
                    background: motionType === t ? C.accent : C.panelAlt,
                    color: motionType === t ? "#fff" : C.sub,
                    border: `1px solid ${motionType === t ? C.accent : C.borderSoft}`,
                  }}
                >
                  {t}
                </button>
              ))}
            </div>
          </div>

          {/* Joint value inputs */}
          <div className="flex flex-col gap-2">
            {["j1", "j2", "j3", "j4", "j5"].map((key, i) => (
              <div key={key} className="flex items-center justify-between gap-2">
                <span className="text-xs shrink-0" style={{ color: C.sub }}>
                  {`J${i + 1} (${key === "j5" ? "%" : "°"})`}
                </span>
                <input
                  type="text"
                  inputMode="decimal"
                  value={key === "j4" ? "90" : jointInputs[key]}
                  onChange={(e) => handleJointInputChange(key, e.target.value)}
                  disabled={key === "j4"}
                  className="flex-1 min-w-0 px-2.5 py-1.5 rounded-lg text-xs text-right outline-none"
                  style={{
                    background: C.panelAlt,
                    border: `1px solid ${C.borderSoft}`,
                    color: key === "j4" ? C.subDim : C.text,
                    opacity: key === "j4" ? 0.6 : 1,
                  }}
                />
              </div>
            ))}
          </div>

          {/* Move button */}
          <button
            onClick={handleMovePTP}
            disabled={!modelReady || isMoving || isPlayingAll}
            className="flex items-center justify-center gap-2 py-2.5 rounded-xl text-[13px] font-semibold transition-colors"
            style={{
              background: !modelReady || isMoving || isPlayingAll ? C.panelAlt : C.accent,
              color: !modelReady || isMoving || isPlayingAll ? C.subDim : "#fff",
              cursor: !modelReady || isMoving || isPlayingAll ? "default" : "pointer",
            }}
          >
            <Move size={14} />
            {isMoving && !isPlayingAll ? "กำลังเคลื่อนที่..." : `Move (${motionType})`}
          </button>

          {/* ---- Trail recording controls ---- */}
          <div style={{ borderTop: `1px solid ${C.borderSoft}`, paddingTop: 12 }}>
            <div className="text-[10px] font-semibold tracking-wide mb-2" style={{ color: C.subDim }}>
              MOTION TRAIL
            </div>
            <div className="flex gap-2">
              <button
                onClick={() => {
                  if (!trailControlRef.current || !modelReady) return;
                  if (trailActive) {
                    trailControlRef.current.stop();
                    setTrailActive(false);
                  } else {
                    trailControlRef.current.start(motionType);
                    setTrailActive(true);
                  }
                }}
                disabled={!modelReady}
                className="flex-1 flex items-center justify-center gap-2 py-2 rounded-xl text-xs font-semibold transition-all"
                style={{
                  background: trailActive
                    ? "rgba(239,68,68,0.15)"
                    : "rgba(34,197,94,0.15)",
                  color: trailActive ? C.red : C.green,
                  border: `1px solid ${trailActive ? C.red : C.green}44`,
                  opacity: !modelReady ? 0.5 : 1,
                }}
              >
                <span
                  className="w-2 h-2 rounded-full"
                  style={{
                    background: trailActive ? C.red : C.green,
                    boxShadow: trailActive ? `0 0 6px ${C.red}` : "none",
                    animation: trailActive ? "pulse 1s infinite" : "none",
                  }}
                />
                {trailActive ? "หยุดบันทึก" : "บันทึกเส้นทาง"}
              </button>
              <button
                onClick={() => {
                  trailControlRef.current?.clear();
                  setTrailCount(0);
                  setTrailActive(false);
                }}
                disabled={!modelReady}
                title="ล้างเส้นทาง"
                className="px-3 py-2 rounded-xl text-xs transition-colors"
                style={{
                  background: C.panelAlt,
                  color: C.sub,
                  border: `1px solid ${C.borderSoft}`,
                  opacity: !modelReady ? 0.5 : 1,
                }}
              >
                <RotateCcw size={13} />
              </button>
            </div>
            {trailActive && (
              <div className="mt-2 flex items-center gap-1.5">
                <span className="w-1.5 h-1.5 rounded-full" style={{ background: C.red, boxShadow: `0 0 4px ${C.red}` }} />
                <span className="text-[10px]" style={{ color: C.subDim }}>กำลังบันทึก — เคลื่อนแขนเพื่อวาดเส้น</span>
              </div>
            )}
          </div>

          {/* Save current pose */}
          <button
            onClick={handleSavePose}
            disabled={!modelReady || isPlayingAll}
            className="flex items-center justify-center gap-2 py-2 rounded-xl text-xs font-medium transition-colors"
            style={{
              background: C.panelAlt,
              border: `1px solid ${C.borderSoft}`,
              color: !modelReady || isPlayingAll ? C.subDim : C.text,
              cursor: !modelReady || isPlayingAll ? "default" : "pointer",
              opacity: !modelReady || isPlayingAll ? 0.6 : 1,
            }}
          >
            <Save size={13} />
            บันทึกท่าทาง
          </button>

          {/* Saved poses list */}
          {savedPoses.length > 0 && (
            <div className="flex flex-col gap-1.5 -mt-1">
              <div className="flex items-center justify-between">
                <div className="text-[10px] font-semibold tracking-wide" style={{ color: C.subDim }}>
                  ท่าทางที่บันทึกไว้
                </div>
                <button
                  onClick={handlePlayAll}
                  disabled={!modelReady || isMoving || isPlayingAll}
                  title="เล่นท่าทางทั้งหมดตามลำดับ (เริ่มจากตำแหน่งเริ่มต้นอัตโนมัติ)"
                  className="flex items-center gap-1 px-2 py-1 rounded-md text-[10px] font-medium transition-colors"
                  style={{
                    background: C.accentSoft,
                    color: C.accent,
                    opacity: !modelReady || isMoving || isPlayingAll ? 0.5 : 1,
                  }}
                >
                  <Play size={10} />
                  {isPlayingAll ? "กำลังเล่น..." : "เล่นทั้งหมด"}
                </button>
              </div>
              <div className="flex flex-col gap-1.5 max-h-40 overflow-y-auto pr-0.5">
                {savedPoses.map((pose) => (
                  <div
                    key={pose.id}
                    className="flex items-center gap-1.5 px-2.5 py-1.5 rounded-lg"
                    style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}
                  >
                    <span className="flex-1 min-w-0 truncate text-xs" style={{ color: C.text }}>
                      {pose.name}
                    </span>
                    <button
                      onClick={() => handleGoToPose(pose)}
                      disabled={!modelReady || isMoving || isPlayingAll}
                      title="ไปยังท่านี้"
                      className="p-1 rounded-md transition-colors"
                      style={{ background: C.accentSoft, color: C.accent, opacity: !modelReady || isMoving || isPlayingAll ? 0.5 : 1 }}
                    >
                      <Play size={11} />
                    </button>
                    <button
                      onClick={() => handleDeletePose(pose.id)}
                      disabled={isPlayingAll}
                      title="ลบท่านี้"
                      className="p-1 rounded-md transition-colors"
                      style={{ background: "rgba(239,68,68,0.12)", color: C.red, opacity: isPlayingAll ? 0.5 : 1 }}
                    >
                      <Trash2 size={11} />
                    </button>
                  </div>
                ))}
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
