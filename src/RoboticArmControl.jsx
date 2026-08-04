import React, { useEffect, useRef, useState, useCallback, useMemo } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import robotModel from "./assets/arm_robotics.glb";
import {
  Bot, Wifi, ChevronDown, Home as HomeIcon, Move, Save, Trash2,
  Play, Pause, Repeat, Activity, RotateCcw, BookOpen,
} from "lucide-react";

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
  amber: "#f59e0b",
  red: "#ef4444",
};

// ---------------------------------------------------------------------------
// Robot arm link lengths (เมตร) — ใช้แค่ J1 (ฐาน) / J2 (ไหล่) / J3 (ข้อศอก)
// ---------------------------------------------------------------------------
const L1 = 0.10; // ความสูงไหล่จากพื้น (m)
const L2 = 0.105; // แขนบน ไหล่-ข้อศอก (m)
const L3 = 0.096; // แขนล่าง ข้อศอก-ข้อมือ (m)

const HOME = { j1: 0, j2: 0, j3: 90, j4: 90, j5: 0 };

/** solveIK3 — Inverse Kinematics แบบง่าย (2-link planar + ฐานหมุน) หา J1/J2/J3 */
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

/** fk3 — Forward Kinematics คู่กับ solveIK3 */
function fk3(j1Deg, j2Deg, j3Deg) {
  const j1 = THREE.MathUtils.degToRad(j1Deg);
  const j2 = THREE.MathUtils.degToRad(j2Deg);
  const j3Mag = THREE.MathUtils.degToRad(Math.abs(j3Deg));
  const dist = Math.sqrt(L2 * L2 + L3 * L3 + 2 * L2 * L3 * Math.cos(j3Mag));
  const sinBetaMag = dist > 1e-9 ? THREE.MathUtils.clamp((L3 * Math.sin(j3Mag)) / dist, -1, 1) : 0;
  const betaMag = Math.asin(sinBetaMag);
  const alpha = j2 - betaMag;
  const r = dist * Math.cos(alpha);
  const dy = dist * Math.sin(alpha);
  const y = dy + L1;
  const x = r * Math.sin(j1);
  const z = r * Math.cos(j1);
  return new THREE.Vector3(x, y, z);
}

/** computeArc — หาวงกลมที่ผ่าน 3 จุด (P0 → Pv → P1) ในพิกัด 3 มิติ */
function computeArc(P0, P1, Pv) {
  const AC = new THREE.Vector3().subVectors(P1, P0);
  const AB = new THREE.Vector3().subVectors(Pv, P0);
  const normal = new THREE.Vector3().crossVectors(AB, AC);
  if (normal.lengthSq() < 1e-10) return null;
  normal.normalize();

  const d = AC.length();
  if (d < 1e-6) return null;
  const u = AC.clone().multiplyScalar(1 / d);
  const v = new THREE.Vector3().crossVectors(normal, u).normalize();

  const ax = 0, ay = 0;
  const bx = d, by = 0;
  const cx = AB.dot(u), cy = AB.dot(v);

  const D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (Math.abs(D) < 1e-9) return null;
  const ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / D;
  const uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / D;

  const center = P0.clone().add(u.clone().multiplyScalar(ux)).add(v.clone().multiplyScalar(uy));
  const R = Math.sqrt((ax - ux) ** 2 + (ay - uy) ** 2);

  const angle0 = Math.atan2(ay - uy, ax - ux);
  const angle1 = Math.atan2(by - uy, bx - ux);
  const angleV = Math.atan2(cy - uy, cx - ux);

  const TWO_PI = Math.PI * 2;
  const norm = (a) => ((a % TWO_PI) + TWO_PI) % TWO_PI;
  const sweepCCW = norm(angle1 - angle0);
  const deltaV = norm(angleV - angle0);
  const sweep = deltaV <= sweepCCW ? sweepCCW : sweepCCW - TWO_PI;

  return { center, u, v, R, angle0, sweep };
}

function arcPoint(arc, t) {
  const angle = arc.angle0 + arc.sweep * t;
  return arc.center
    .clone()
    .add(arc.u.clone().multiplyScalar(arc.R * Math.cos(angle)))
    .add(arc.v.clone().multiplyScalar(arc.R * Math.sin(angle)));
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

    const MAX_TRAIL_POINTS = 4000;
    const trailPositions = new Float32Array(MAX_TRAIL_POINTS * 3);
    const trailColors = new Float32Array(MAX_TRAIL_POINTS * 3);
    let trailCount = 0;
    let lastTrailPos = null;
    const MIN_DIST = 0.004;

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

    const waypointDots = [];

    function addTrailPoint(pos, motionType) {
      if (trailCount >= MAX_TRAIL_POINTS) return;
      const i = trailCount * 3;
      trailPositions[i] = pos.x;
      trailPositions[i + 1] = pos.y;
      trailPositions[i + 2] = pos.z;

      const col = motionType === "LIN"
        ? new THREE.Color(0x22c55e)
        : motionType === "CIRC"
        ? new THREE.Color(0xf59e0b)
        : new THREE.Color(0x3b6cf6);
      trailColors[i] = col.r;
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

    const trailState = { recording: false, motionType: "PTP" };
    if (trailControlRef) {
      trailControlRef.current = {
        start: (mt) => { trailState.recording = true; trailState.motionType = mt || "PTP"; if (trailCount > 0 && lastTrailPos) addWaypointDot(lastTrailPos, mt); },
        stop: () => { trailState.recording = false; if (lastTrailPos) addWaypointDot(lastTrailPos, "PTP"); },
        clear: clearTrail,
        getCount: () => trailCount,
      };
    }

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

    const miniScene = new THREE.Scene();
    const miniGrid = new THREE.GridHelper(0.7, 14, 0x2c3766, 0x1c2340);
    miniGrid.material.opacity = 0.7;
    miniGrid.material.transparent = true;
    miniScene.add(miniGrid);
    miniScene.add(new THREE.AxesHelper(0.12));

    const miniTrailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.95,
    }));
    miniTrailLine.frustumCulled = false;
    miniScene.add(miniTrailLine);

    const curDotGeo = new THREE.CircleGeometry(0.012, 10);
    const curDotMat = new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false });
    const curDot = new THREE.Mesh(curDotGeo, curDotMat);
    curDot.rotation.x = -Math.PI / 2;
    miniScene.add(curDot);

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

          if (trailState.recording) {
            if (!lastTrailPos || _wristWorld.distanceTo(lastTrailPos) >= MIN_DIST) {
              addTrailPoint(_wristWorld, trailState.motionType);
              lastTrailPos = _wristWorld.clone();
            }
          }
          curDot.position.set(_wristWorld.x, 0.001, _wristWorld.z);
        }
        const camDist = camera.position.distanceTo(s.gizmo.position);
        s.gizmo.scale.setScalar(camDist / GIZMO_REF_DISTANCE);
      }
      renderer.render(scene, camera);
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

  useEffect(() => {
    const s = sceneRef.current;
    if (!s || !s.ready) return;
    const d = THREE.MathUtils.degToRad;
    s.baseGroup.rotation.y = d(joints.j1);
    s.shoulder.rotation.x = d(-joints.j2);
    s.elbow.rotation.y = d(-joints.j3);
    s.wrist.rotation.y = d(-joints.j4);
    s.baseGroup.updateMatrixWorld(true);
  }, [joints]);

  return { modelReady, modelError };
}

// ---------------------------------------------------------------------------
// Left icon rail — navigation between "Home" (control) and "Learn" pages
// ---------------------------------------------------------------------------
function IconRail({ page, setPage }) {
  const items = [
    { id: "control", icon: HomeIcon, title: "แผงควบคุม" },
    { id: "learn", icon: BookOpen, title: "เรียนรู้ Motion" },
  ];
  return (
    <div
      className="flex flex-col items-center gap-2 py-4 shrink-0"
      style={{ width: 56, background: C.panel, borderRight: `1px solid ${C.border}` }}
    >
      {items.map(({ id, icon: Icon, title }) => {
        const active = page === id;
        return (
          <button
            key={id}
            onClick={() => setPage(id)}
            title={title}
            className="w-10 h-10 rounded-xl flex items-center justify-center transition-colors"
            style={{
              background: active ? C.accentSoft : "transparent",
              color: active ? C.accent : C.subDim,
              border: `1px solid ${active ? C.accent + "55" : "transparent"}`,
            }}
          >
            <Icon size={18} />
          </button>
        );
      })}
    </div>
  );
}

// ---------------------------------------------------------------------------
// J1–J5 value bar — spans the top of the viewer, mirrors the wireframe header
// ---------------------------------------------------------------------------
function JointValueBar({ joints }) {
  const rows = [
    ["J1", joints.j1, "°"],
    ["J2", joints.j2, "°"],
    ["J3", joints.j3, "°"],
    ["J4", joints.j4, "°"],
    ["J5", joints.j5, "%"],
  ];
  return (
    <div
      className="flex items-center shrink-0"
      style={{ background: C.panelAlt, borderBottom: `1px solid ${C.border}` }}
    >
      {rows.map(([label, val, unit]) => (
        <div key={label} className="flex-1 flex items-baseline justify-center gap-1.5 py-2.5" style={{ borderRight: `1px solid ${C.borderSoft}` }}>
          <span className="text-[11px] font-semibold" style={{ color: C.subDim }}>{label}</span>
          <span className="text-[13px] font-mono font-semibold" style={{ color: C.text }}>
            {typeof val === "number" ? val.toFixed(1) : val}{unit}
          </span>
        </div>
      ))}
    </div>
  );
}

// ---------------------------------------------------------------------------
// LEARN PAGE — แขนกล 2D สำหรับสอนเรื่อง Motion: PTP / LIN / CIRC + สูตรคำนวณ
// ---------------------------------------------------------------------------
const LEARN_L1 = 90; // px, ความยาวแขนบน (สเกลสำหรับวาด 2D เท่านั้น)
const LEARN_L2 = 80; // px, ความยาวแขนล่าง
const LEARN_ORIGIN = { x: 170, y: 230 };

function learnFk(theta1Deg, theta2Deg) {
  const t1 = THREE.MathUtils.degToRad(theta1Deg);
  const t2 = THREE.MathUtils.degToRad(theta2Deg);
  const elbow = {
    x: LEARN_ORIGIN.x + LEARN_L1 * Math.cos(t1),
    y: LEARN_ORIGIN.y - LEARN_L1 * Math.sin(t1),
  };
  const wrist = {
    x: elbow.x + LEARN_L2 * Math.cos(t1 + t2),
    y: elbow.y - LEARN_L2 * Math.sin(t1 + t2),
  };
  return { elbow, wrist };
}

function learnIk(x, y) {
  const dx = x - LEARN_ORIGIN.x;
  const dy = LEARN_ORIGIN.y - y;
  const dist = Math.sqrt(dx * dx + dy * dy);
  const distClamped = THREE.MathUtils.clamp(dist, Math.abs(LEARN_L1 - LEARN_L2) + 1, LEARN_L1 + LEARN_L2 - 1);
  const cosT2 = (distClamped * distClamped - LEARN_L1 * LEARN_L1 - LEARN_L2 * LEARN_L2) / (2 * LEARN_L1 * LEARN_L2);
  const t2 = Math.acos(THREE.MathUtils.clamp(cosT2, -1, 1));
  const k1 = LEARN_L1 + LEARN_L2 * Math.cos(t2);
  const k2 = LEARN_L2 * Math.sin(t2);
  const t1 = Math.atan2(dy, dx) - Math.atan2(k2, k1);
  return {
    theta1: THREE.MathUtils.radToDeg(t1),
    theta2: THREE.MathUtils.radToDeg(t2),
  };
}

const LEARN_POSE_A = { theta1: 40, theta2: 70 };
const LEARN_POSE_B = { theta1: 130, theta2: -60 };
const LEARN_VIA = { x: LEARN_ORIGIN.x, y: LEARN_ORIGIN.y - (LEARN_L1 + LEARN_L2) * 0.72 };

const MOTION_INFO = {
  PTP: {
    color: C.accent,
    name: "PTP — Point to Point",
    tagline: "สอดแทรกในปริภูมิข้อต่อ (Joint Space)",
    desc: "แต่ละข้อต่อ (θ1, θ2) เคลื่อนที่จากมุมเริ่มต้นไปยังมุมเป้าหมายพร้อมกันแบบเชิงเส้น โดยไม่สนใจว่าปลายแขนจะเคลื่อนที่เป็นเส้นทางแบบใด ทำให้เร็วและประหยัดพลังงานที่สุด แต่เส้นทางของปลายแขนจะโค้งงอ ไม่ใช่เส้นตรง",
    formula: "θᵢ(t) = θᵢ₀ + (θᵢ₁ − θᵢ₀) · t,   t ∈ [0,1]",
    note: "ใช้กับทุกข้อต่อพร้อมกัน (i = 1..n) — นี่คือสูตรง่ายที่สุดเพราะไม่ต้องคำนวณ IK ระหว่างทางเลย",
  },
  LIN: {
    color: C.green,
    name: "LIN — Linear",
    tagline: "สอดแทรกในปริภูมิคาร์ทีเซียน (Cartesian Space)",
    desc: "ปลายแขนถูกบังคับให้เคลื่อนที่เป็นเส้นตรงจากจุดเริ่มต้น P0 ไปยังจุดปลาย P1 เสมอ โดยคำนวณตำแหน่ง P(t) ก่อน แล้วจึงแปลงกลับเป็นมุมข้อต่อด้วย Inverse Kinematics (IK) ทุกเฟรม — ใช้ในงานที่ต้องการความแม่นยำของเส้นทาง เช่น การเชื่อมหรือตัดวัสดุ",
    formula: "P(t) = P0 + (P1 − P0) · t   แล้วแก้ IK: cos θ2 = (d² − L1² − L2²) / (2 L1 L2)",
    note: "d = ระยะจากฐานถึง P(t), L1/L2 = ความยาวท่อนแขน — มุม θ1 หาได้จาก atan2 ของตำแหน่งและมุม θ2",
  },
  CIRC: {
    color: C.amber,
    name: "CIRC — Circular",
    tagline: "สอดแทรกตามส่วนโค้งของวงกลมผ่าน 3 จุด",
    desc: "กำหนดจุดเริ่ม P0, จุดผ่าน (via point) Pv และจุดปลาย P1 ระบบจะหาวงกลมที่ลากผ่านทั้ง 3 จุดนี้ได้พอดี แล้วสอดแทรกตำแหน่งไปตามส่วนโค้งนั้น ก่อนแปลงกลับเป็นมุมข้อต่อด้วย IK เช่นเดียวกับ LIN — ใช้เมื่อต้องการเลี่ยงสิ่งกีดขวางด้วยเส้นทางโค้ง",
    formula: "หาศูนย์กลางวงกลม C และรัศมี R จาก P0,P1,Pv  →  P(t) = C + R·(cos φ(t), sin φ(t))",
    note: "φ(t) กวาดมุมจาก φ0 ไป φ0+Δφ ตามทิศที่ผ่าน Pv จริง (เลือกทิศตามเข็ม/ทวนเข็มโดยอัตโนมัติ)",
  },
};

function Learn2DArm({ pose, trail, motionType }) {
  const { elbow, wrist } = learnFk(pose.theta1, pose.theta2);
  const info = MOTION_INFO[motionType];
  return (
    <svg viewBox="0 0 340 260" className="w-full h-full">
      {Array.from({ length: 8 }).map((_, i) => (
        <line key={"v" + i} x1={i * 42} y1={0} x2={i * 42} y2={260} stroke={C.borderSoft} strokeWidth={1} />
      ))}
      {Array.from({ length: 6 }).map((_, i) => (
        <line key={"h" + i} x1={0} y1={i * 45} x2={340} y2={i * 45} stroke={C.borderSoft} strokeWidth={1} />
      ))}

      {trail.length > 1 && (
        <polyline
          points={trail.map((p) => `${p.x},${p.y}`).join(" ")}
          fill="none"
          stroke={info.color}
          strokeWidth={2.5}
          strokeLinecap="round"
          opacity={0.85}
        />
      )}

      {motionType === "CIRC" && (
        <g>
          <circle cx={LEARN_VIA.x} cy={LEARN_VIA.y} r={4} fill={C.amber} />
          <text x={LEARN_VIA.x + 8} y={LEARN_VIA.y - 6} fontSize="9" fill={C.amber} fontFamily="monospace">via</text>
        </g>
      )}

      {[LEARN_POSE_A, LEARN_POSE_B].map((p, i) => {
        const { wrist: w } = learnFk(p.theta1, p.theta2);
        return <circle key={i} cx={w.x} cy={w.y} r={3} fill={C.subDim} />;
      })}

      <circle cx={LEARN_ORIGIN.x} cy={LEARN_ORIGIN.y} r={7} fill={C.panelAlt} stroke={C.border} strokeWidth={2} />
      <line x1={LEARN_ORIGIN.x} y1={LEARN_ORIGIN.y} x2={elbow.x} y2={elbow.y} stroke={C.text} strokeWidth={6} strokeLinecap="round" />
      <line x1={elbow.x} y1={elbow.y} x2={wrist.x} y2={wrist.y} stroke={C.sub} strokeWidth={6} strokeLinecap="round" />
      <circle cx={elbow.x} cy={elbow.y} r={5} fill={C.accent} />
      <circle cx={wrist.x} cy={wrist.y} r={6} fill={info.color} stroke={C.bg} strokeWidth={2} />
    </svg>
  );
}

function LearnPage() {
  const [motionType, setMotionType] = useState("PTP");
  const [pose, setPose] = useState(LEARN_POSE_A);
  const [trail, setTrail] = useState([]);
  const [playing, setPlaying] = useState(false);
  const animRef = useRef(null);

  const runDemo = useCallback(() => {
    if (playing) return;
    if (animRef.current) cancelAnimationFrame(animRef.current);
    setTrail([]);
    setPlaying(true);

    const start = LEARN_POSE_A;
    const end = LEARN_POSE_B;
    const P0 = learnFk(start.theta1, start.theta2).wrist;
    const P1 = learnFk(end.theta1, end.theta2).wrist;

    let pathFn;
    if (motionType === "PTP") {
      pathFn = null;
    } else if (motionType === "LIN") {
      pathFn = (t) => ({ x: P0.x + (P1.x - P0.x) * t, y: P0.y + (P1.y - P0.y) * t });
    } else {
      const arc2D = (() => {
        const ax = P0.x, ay = P0.y, bx = LEARN_VIA.x, by = LEARN_VIA.y, cx = P1.x, cy = P1.y;
        const D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        const ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / D;
        const uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / D;
        const R = Math.hypot(ax - ux, ay - uy);
        const a0 = Math.atan2(ay - uy, ax - ux);
        const a1 = Math.atan2(cy - uy, cx - ux);
        const av = Math.atan2(by - uy, bx - ux);
        const TWO_PI = Math.PI * 2;
        const norm = (a) => ((a % TWO_PI) + TWO_PI) % TWO_PI;
        const sweepCCW = norm(a1 - a0);
        const deltaV = norm(av - a0);
        const sweep = deltaV <= sweepCCW ? sweepCCW : sweepCCW - TWO_PI;
        return { ux, uy, R, a0, sweep };
      })();
      pathFn = (t) => {
        const ang = arc2D.a0 + arc2D.sweep * t;
        return { x: arc2D.ux + arc2D.R * Math.cos(ang), y: arc2D.uy + arc2D.R * Math.sin(ang) };
      };
    }

    const duration = motionType === "PTP" ? 1000 : motionType === "LIN" ? 1500 : 1400;
    const startTime = performance.now();
    const ease = (t) => 1 - Math.pow(1 - t, 3);

    function step(now) {
      const t = Math.min(1, (now - startTime) / duration);
      const e = ease(t);
      let nextPose, wristPos;
      if (motionType === "PTP") {
        nextPose = {
          theta1: start.theta1 + (end.theta1 - start.theta1) * e,
          theta2: start.theta2 + (end.theta2 - start.theta2) * e,
        };
        wristPos = learnFk(nextPose.theta1, nextPose.theta2).wrist;
      } else {
        wristPos = pathFn(e);
        nextPose = learnIk(wristPos.x, wristPos.y);
      }
      setPose(nextPose);
      setTrail((prev) => (prev.length && Math.hypot(prev[prev.length - 1].x - wristPos.x, prev[prev.length - 1].y - wristPos.y) < 1 ? prev : [...prev, wristPos]));
      if (t < 1) {
        animRef.current = requestAnimationFrame(step);
      } else {
        animRef.current = null;
        setPlaying(false);
      }
    }
    animRef.current = requestAnimationFrame(step);
  }, [motionType, playing]);

  useEffect(() => () => { if (animRef.current) cancelAnimationFrame(animRef.current); }, []);

  const info = MOTION_INFO[motionType];

  return (
    <div className="flex-1 min-h-0 overflow-y-auto p-6" style={{ background: C.bg }}>
      <div className="max-w-5xl mx-auto flex flex-col gap-5">
        <div>
          <div className="flex items-center gap-2 mb-1">
            <BookOpen size={16} color={C.accent} />
            <span className="text-[15px] font-semibold" style={{ color: C.text }}>เรียนรู้ Motion: PTP / LIN / CIRC</span>
          </div>
          <p className="text-xs" style={{ color: C.sub }}>
            แขนกลจำลอง 2 ข้อต่อแบบระนาบ (2D) สำหรับสาธิตความแตกต่างของการสอดแทรกเส้นทางแต่ละแบบ พร้อมสูตรคำนวณประกอบ
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-5">
          <div className="rounded-2xl p-4 flex flex-col gap-3" style={{ background: C.panel, border: `1px solid ${C.border}` }}>
            <div className="flex gap-1.5">
              {["PTP", "LIN", "CIRC"].map((t) => (
                <button
                  key={t}
                  onClick={() => { setMotionType(t); setTrail([]); setPose(LEARN_POSE_A); }}
                  disabled={playing}
                  className="flex-1 py-1.5 rounded-lg text-xs font-medium transition-colors"
                  style={{
                    background: motionType === t ? MOTION_INFO[t].color : C.panelAlt,
                    color: motionType === t ? "#fff" : C.sub,
                    border: `1px solid ${motionType === t ? MOTION_INFO[t].color : C.borderSoft}`,
                    opacity: playing && motionType !== t ? 0.5 : 1,
                  }}
                >
                  {t}
                </button>
              ))}
            </div>
            <div className="rounded-xl overflow-hidden" style={{ background: C.panelAlt, aspectRatio: "340/260" }}>
              <Learn2DArm pose={pose} trail={trail} motionType={motionType} />
            </div>
            <button
              onClick={runDemo}
              disabled={playing}
              className="flex items-center justify-center gap-2 py-2.5 rounded-xl text-[13px] font-semibold transition-colors"
              style={{
                background: playing ? C.panelAlt : info.color,
                color: playing ? C.subDim : "#fff",
                cursor: playing ? "default" : "pointer",
              }}
            >
              <Play size={14} />
              {playing ? "กำลังสาธิต..." : `สาธิตการเคลื่อนที่แบบ ${motionType}`}
            </button>
          </div>

          <div className="rounded-2xl p-5 flex flex-col gap-3" style={{ background: C.panel, border: `1px solid ${C.border}` }}>
            <div className="flex items-center gap-2">
              <span className="w-2.5 h-2.5 rounded-full" style={{ background: info.color }} />
              <span className="text-sm font-semibold" style={{ color: C.text }}>{info.name}</span>
            </div>
            <span className="text-[11px] font-medium" style={{ color: info.color }}>{info.tagline}</span>
            <p className="text-xs leading-relaxed" style={{ color: C.sub }}>{info.desc}</p>

            <div className="rounded-xl p-3 mt-1" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}>
              <div className="text-[10px] font-semibold tracking-wide mb-1.5" style={{ color: C.subDim }}>สูตรคำนวณ</div>
              <div className="text-[13px] font-mono" style={{ color: C.text }}>{info.formula}</div>
              <div className="text-[10px] mt-2 leading-relaxed" style={{ color: C.subDim }}>{info.note}</div>
            </div>

            <div className="grid grid-cols-3 gap-2 mt-1">
              {Object.entries(MOTION_INFO).map(([key, m]) => (
                <div key={key} className="rounded-lg px-2.5 py-2 text-center" style={{ background: C.panelAlt, border: `1px solid ${key === motionType ? m.color : C.borderSoft}` }}>
                  <div className="w-1.5 h-1.5 rounded-full mx-auto mb-1" style={{ background: m.color }} />
                  <div className="text-[10px] font-semibold" style={{ color: C.text }}>{key}</div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

// ---------------------------------------------------------------------------
// CONTROL PAGE — reorganized per wireframe:
// Home button → Motion type → Motion Trail toggle → play/loop/pause → pose list
// ---------------------------------------------------------------------------
function ControlPanel({
  modelReady, isMoving, isPlayingAll, handleHome,
  motionType, setMotionType,
  viaInputs, handleViaInputChange,
  jointInputs, handleJointInputChange, handleMove,
  trailActive, trailControlRef, setTrailActive, setTrailCount,
  savedPoses, handleSavePose, handleGoToPose, handleDeletePose, handlePlayAll,
}) {
  return (
    <div
      className="w-[280px] shrink-0 flex flex-col gap-4 p-4 overflow-y-auto"
      style={{ background: C.panel, borderRight: `1px solid ${C.border}` }}
    >
      <button
        onClick={handleHome}
        disabled={!modelReady || isMoving || isPlayingAll}
        className="flex items-center justify-center gap-2 py-2.5 rounded-xl text-[13px] font-semibold transition-colors"
        style={{
          background: !modelReady || isMoving || isPlayingAll ? C.panelAlt : C.accentSoft,
          color: !modelReady || isMoving || isPlayingAll ? C.subDim : C.accent,
          border: `1px solid ${C.borderSoft}`,
          cursor: !modelReady || isMoving || isPlayingAll ? "default" : "pointer",
        }}
      >
        <HomeIcon size={14} />
        Home
      </button>

      <div>
        <div className="text-[10px] font-semibold tracking-wide mb-1.5" style={{ color: C.subDim }}>MOTION</div>
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

      {motionType === "CIRC" && (
        <div>
          <div className="text-[10px] font-semibold tracking-wide mb-1.5" style={{ color: C.subDim }}>VIA POINT (ม., ไม่บังคับ)</div>
          <div className="flex gap-1.5">
            {["x", "y", "z"].map((axis) => (
              <input
                key={axis}
                type="text"
                inputMode="decimal"
                placeholder={axis.toUpperCase()}
                value={viaInputs[axis]}
                onChange={(e) => handleViaInputChange(axis, e.target.value)}
                className="flex-1 min-w-0 px-2 py-1.5 rounded-lg text-xs text-center outline-none"
                style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text }}
              />
            ))}
          </div>
        </div>
      )}

      <div className="flex items-center justify-between">
        <span className="text-[10px] font-semibold tracking-wide" style={{ color: C.subDim }}>MOTION TRAIL</span>
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
          className="relative rounded-full transition-colors"
          style={{ width: 40, height: 22, background: trailActive ? C.green : C.borderSoft, opacity: !modelReady ? 0.5 : 1 }}
        >
          <span
            className="absolute top-[3px] rounded-full transition-all"
            style={{ width: 16, height: 16, background: "#fff", left: trailActive ? 21 : 3 }}
          />
        </button>
      </div>

      <div className="flex items-center justify-center gap-3 py-1">
        <button
          onClick={handlePlayAll}
          disabled={!modelReady || isMoving || isPlayingAll || savedPoses.length === 0}
          title="เล่นท่าทางทั้งหมดตามลำดับ"
          className="w-9 h-9 rounded-full flex items-center justify-center"
          style={{ background: C.accentSoft, color: C.accent, opacity: !modelReady || isMoving || isPlayingAll || savedPoses.length === 0 ? 0.4 : 1 }}
        >
          <Play size={15} />
        </button>
        <button
          onClick={() => { trailControlRef.current?.clear(); setTrailCount(0); setTrailActive(false); }}
          disabled={!modelReady}
          title="วนซ้ำ / ล้างเส้นทาง"
          className="w-9 h-9 rounded-full flex items-center justify-center"
          style={{ background: C.panelAlt, color: C.sub, border: `1px solid ${C.borderSoft}`, opacity: !modelReady ? 0.5 : 1 }}
        >
          <Repeat size={14} />
        </button>
        <button
          disabled
          title="หยุดชั่วคราว"
          className="w-9 h-9 rounded-full flex items-center justify-center"
          style={{ background: C.panelAlt, color: C.subDim, border: `1px solid ${C.borderSoft}`, opacity: 0.5 }}
        >
          <Pause size={14} />
        </button>
      </div>

      <div className="flex flex-col gap-1.5">
        <div className="text-[10px] font-semibold tracking-wide" style={{ color: C.subDim }}>POSE</div>
        <div className="flex flex-col gap-1.5 max-h-60 overflow-y-auto pr-0.5">
          {savedPoses.length === 0 && (
            <div className="text-[11px] text-center py-3" style={{ color: C.subDim }}>ยังไม่มีท่าทางที่บันทึกไว้</div>
          )}
          {savedPoses.map((pose) => (
            <div key={pose.id} className="flex items-center gap-1.5 px-2.5 py-1.5 rounded-lg" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}` }}>
              <span className="flex-1 min-w-0 truncate text-xs" style={{ color: C.text }}>{pose.name}</span>
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
        <button
          onClick={handleSavePose}
          disabled={!modelReady || isPlayingAll}
          className="flex items-center justify-center gap-2 py-2 rounded-xl text-xs font-medium transition-colors mt-1"
          style={{
            background: C.panelAlt,
            border: `1px solid ${C.borderSoft}`,
            color: !modelReady || isPlayingAll ? C.subDim : C.text,
            opacity: !modelReady || isPlayingAll ? 0.6 : 1,
          }}
        >
          <Save size={13} />
          บันทึกท่าทางปัจจุบัน
        </button>
      </div>

      <div className="flex flex-col gap-2 pt-2" style={{ borderTop: `1px solid ${C.borderSoft}` }}>
        <div className="text-[10px] font-semibold tracking-wide" style={{ color: C.subDim }}>MOVE TO JOINTS</div>
        {["j1", "j2", "j3", "j4", "j5"].map((key, i) => (
          <div key={key} className="flex items-center justify-between gap-2">
            <span className="text-xs shrink-0" style={{ color: C.sub }}>{`J${i + 1} (${key === "j5" ? "%" : "°"})`}</span>
            <input
              type="text"
              inputMode="decimal"
              value={key === "j4" ? "90" : jointInputs[key]}
              onChange={(e) => handleJointInputChange(key, e.target.value)}
              disabled={key === "j4"}
              className="flex-1 min-w-0 px-2.5 py-1.5 rounded-lg text-xs text-right outline-none"
              style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: key === "j4" ? C.subDim : C.text, opacity: key === "j4" ? 0.6 : 1 }}
            />
          </div>
        ))}
        <button
          onClick={handleMove}
          disabled={!modelReady || isMoving || isPlayingAll}
          className="flex items-center justify-center gap-2 py-2.5 rounded-xl text-[13px] font-semibold transition-colors mt-1"
          style={{
            background: !modelReady || isMoving || isPlayingAll ? C.panelAlt : C.accent,
            color: !modelReady || isMoving || isPlayingAll ? C.subDim : "#fff",
            cursor: !modelReady || isMoving || isPlayingAll ? "default" : "pointer",
          }}
        >
          <Move size={14} />
          {isMoving && !isPlayingAll ? "กำลังเคลื่อนที่..." : `Move (${motionType})`}
        </button>
      </div>
    </div>
  );
}

// ---------------------------------------------------------------------------
// Main component
// ---------------------------------------------------------------------------
export default function RoboticArmControl() {
  const [page, setPage] = useState("control"); // "control" | "learn"

  const [ports, setPorts] = useState(["COM3"]);
  const [selectedPort, setSelectedPort] = useState("COM3");
  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);

  const trailControlRef = useRef(null);
  const [trailActive, setTrailActive] = useState(false);
  const [trailCount, setTrailCount] = useState(0);

  const [joints, setJoints] = useState(HOME);
  const jointsRef = useRef(HOME);
  useEffect(() => { jointsRef.current = joints; }, [joints]);

  const [motionType, setMotionType] = useState("PTP");
  const [jointInputs, setJointInputs] = useState({
    j1: String(HOME.j1), j2: String(HOME.j2), j3: String(HOME.j3), j4: String(HOME.j4), j5: String(HOME.j5),
  });

  const [viaInputs, setViaInputs] = useState({ x: "", y: "", z: "" });
  const handleViaInputChange = useCallback((axis, value) => {
    if (value === "" || value === "-" || /^-?\d*\.?\d*$/.test(value)) {
      setViaInputs((prev) => ({ ...prev, [axis]: value }));
    }
  }, []);

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
    if (value === "" || value === "-" || /^-?\d*\.?\d*$/.test(value)) {
      setJointInputs((prev) => ({ ...prev, [key]: value }));
    }
  }, []);

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

  useEffect(() => () => { if (moveAnimRef.current) cancelAnimationFrame(moveAnimRef.current); }, []);

  const animateCartesian = useCallback((targetJoints, motion, duration = 1100) => {
    return new Promise((resolve) => {
      if (moveAnimRef.current) cancelAnimationFrame(moveAnimRef.current);
      const startJoints = { ...jointsRef.current };
      const P0 = fk3(startJoints.j1, startJoints.j2, startJoints.j3);
      const P1 = fk3(targetJoints.j1, targetJoints.j2, targetJoints.j3);

      let pathFn = (t) => P0.clone().lerp(P1, t);

      if (motion === "CIRC") {
        let viaVec = null;
        const vx = parseFloat(viaInputs.x);
        const vy = parseFloat(viaInputs.y);
        const vz = parseFloat(viaInputs.z);
        if (Number.isFinite(vx) && Number.isFinite(vy) && Number.isFinite(vz)) {
          viaVec = new THREE.Vector3(vx, vy, vz);
        } else {
          const dir = new THREE.Vector3().subVectors(P1, P0);
          const segLen = dir.length();
          if (segLen > 1e-6) {
            dir.normalize();
            let perp = new THREE.Vector3().crossVectors(dir, new THREE.Vector3(0, 1, 0));
            if (perp.lengthSq() < 1e-6) perp = new THREE.Vector3().crossVectors(dir, new THREE.Vector3(1, 0, 0));
            perp.normalize();
            const mid = new THREE.Vector3().addVectors(P0, P1).multiplyScalar(0.5);
            viaVec = mid.add(perp.multiplyScalar(segLen * 0.25));
          }
        }
        if (viaVec) {
          const arc = computeArc(P0, P1, viaVec);
          if (arc) pathFn = (t) => arcPoint(arc, t);
        }
      }

      const startTime = performance.now();
      const easeOutCubic = (t) => 1 - Math.pow(1 - t, 3);
      setIsMoving(true);
      function step(now) {
        const t = Math.min(1, (now - startTime) / duration);
        const e = easeOutCubic(t);
        const pos = pathFn(e);
        const ik = solveIK3(pos.x, pos.y, pos.z);
        const next = {
          j1: ik.ok ? ik.j1 : startJoints.j1 + (targetJoints.j1 - startJoints.j1) * e,
          j2: ik.ok ? ik.j2 : startJoints.j2 + (targetJoints.j2 - startJoints.j2) * e,
          j3: ik.ok ? ik.j3 : startJoints.j3 + (targetJoints.j3 - startJoints.j3) * e,
          j4: 90,
          j5: startJoints.j5 + (targetJoints.j5 - startJoints.j5) * e,
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
  }, [viaInputs]);

  const MOVE_DURATION = { PTP: 900, LIN: 1500, CIRC: 1300 };

  const runMotion = useCallback((targetJoints, motion) => {
    const duration = MOVE_DURATION[motion] ?? 1100;
    const doMove = motion === "PTP"
      ? () => animateToJoints(targetJoints, duration)
      : () => animateCartesian(targetJoints, motion, duration);

    if (trailControlRef.current) {
      trailControlRef.current.start(motion);
      setTrailActive(true);
      return doMove().then(() => {
        trailControlRef.current?.stop();
        setTrailActive(false);
      });
    }
    return doMove();
  }, [animateToJoints, animateCartesian]);

  const handleMove = useCallback(() => {
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
    runMotion(targetJoints, motionType);
  }, [jointInputs, runMotion, motionType]);

  const DEG_PER_UNIT = 400;

  const [savedPoses, setSavedPoses] = useState([]);

  const handleSavePose = useCallback(() => {
    setSavedPoses((prev) => [...prev, { id: Date.now(), name: `ท่าที่ ${prev.length + 1}`, joints: { ...jointsRef.current } }]);
  }, []);

  const handleGoToPose = useCallback((pose) => { runMotion(pose.joints, motionType); }, [runMotion, motionType]);

  const [isPlayingAll, setIsPlayingAll] = useState(false);

  const handlePlayAll = useCallback(async () => {
    if (isPlayingAll || savedPoses.length === 0) return;
    setIsPlayingAll(true);
    await runMotion(HOME, motionType);
    for (const pose of savedPoses) {
      await runMotion(pose.joints, motionType);
    }
    setIsPlayingAll(false);
  }, [isPlayingAll, savedPoses, runMotion, motionType]);

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
        const step = delta * DEG_PER_UNIT;
        next.j2 = moveToward(cur.j2, 0, step);
        next.j3 = moveToward(cur.j3, 0, step);
      } else {
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

  const handleHome = useCallback(() => { animateToJoints(HOME); }, [animateToJoints]);

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

      <div className="flex items-center justify-between px-5 py-3 shrink-0" style={{ background: C.panel, borderBottom: `1px solid ${C.border}` }}>
        <div className="flex items-center gap-2.5">
          <div className="w-8 h-8 rounded-lg flex items-center justify-center" style={{ background: C.accentSoft }}>
            <Bot size={18} color={C.accent} />
          </div>
          <span className="text-[15px] font-semibold" style={{ color: C.text }}>ANR Robot Studio</span>
        </div>

        <div className="flex items-center gap-3">
          <div className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs relative" style={{ background: C.panelAlt, border: `1px solid ${C.borderSoft}`, color: C.text }}>
            <Wifi size={13} color={C.sub} />
            <select
              value={selectedPort}
              onChange={(e) => setSelectedPort(e.target.value)}
              className="bg-transparent outline-none appearance-none pr-4"
              style={{ color: C.text }}
            >
              {ports.map((p) => (<option key={p} value={p} style={{ background: C.panel }}>{p}</option>))}
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

      <div className="flex-1 min-h-0 flex">
        <IconRail page={page} setPage={setPage} />

        {page === "learn" ? (
          <LearnPage />
        ) : (
          <div className="flex-1 min-h-0 flex">
            <ControlPanel
              modelReady={modelReady} isMoving={isMoving} isPlayingAll={isPlayingAll} handleHome={handleHome}
              motionType={motionType} setMotionType={setMotionType}
              viaInputs={viaInputs} handleViaInputChange={handleViaInputChange}
              jointInputs={jointInputs} handleJointInputChange={handleJointInputChange} handleMove={handleMove}
              trailActive={trailActive} trailControlRef={trailControlRef} setTrailActive={setTrailActive} setTrailCount={setTrailCount}
              savedPoses={savedPoses} handleSavePose={handleSavePose} handleGoToPose={handleGoToPose}
              handleDeletePose={handleDeletePose} handlePlayAll={handlePlayAll}
            />

            <div className="flex-1 min-h-0 flex flex-col">
              <JointValueBar joints={joints} />
              <div className="flex-1 min-h-0 p-4">
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

                  {modelReady && (
                    <div className="absolute pointer-events-none" style={{ top: 16, left: 16, width: 180, height: 180, zIndex: 25 }}>
                      <div
                        className="absolute bottom-0 left-0 right-0 flex items-center justify-between px-2.5 py-1.5 rounded-b-xl"
                        style={{ background: "rgba(10,14,26,0.88)", borderTop: `1px solid rgba(28,35,64,0.8)` }}
                      >
                        <div className="flex items-center gap-1.5">
                          <Activity size={9} color={C.accent} />
                          <span style={{ fontSize: 9, fontFamily: "monospace", color: C.sub, fontWeight: 600, letterSpacing: "0.05em" }}>PATH VIEW</span>
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
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
