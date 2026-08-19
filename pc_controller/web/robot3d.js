/* 3D robot view for the RA8P1 omni robot controller.
 * Simplified model from the reference design: hexagonal omni chassis with
 * three kiwi wheels (120°), central lift column, elevating platform with
 * dual arms and a head display. Motion is driven by the current control
 * input (WASD/QE/RF); when live telemetry is available the wheel spin and
 * lift height follow the real robot.
 *
 * Orbit control uses spherical-coordinate drag (per ui-ux-pro-max threejs
 * guidance fallback) so no OrbitControls addon / importmap is required.
 */

import * as THREE from "/assets/vendor/three.module.js";

const hintEl = document.querySelector("#robot-3d .viz-hint");

function showError(text) {
  if (hintEl) {
    hintEl.textContent = text;
    hintEl.style.color = "#FF453A";
    hintEl.style.pointerEvents = "auto";
  }
}

let booted = false;
try {
  boot();
  booted = true;
} catch (err) {
  console.error("robot3d init failed:", err);
  showError("3D 初始化失败：" + (err && err.message ? err.message : err));
}

function boot() {
  const container = document.getElementById("robot-3d");
  const app = window.__robotApp || {
    getMotion: () => ({ vx: 0, vy: 0, omega: 0, lift: 0 }),
    getTelemetry: () => null,
    onThemeChange: () => {},
    currentTheme: () => "light",
  };

  const ARENA_RADIUS = 7;      // clamp for simulated travel
  const LIFT_REST = 1.73;      // platform group height at lift = 0 (flush on the column)
  const LIFT_TRAVEL = 1.2;     // scene units of platform travel
  const MOVE_SPEED = 2.4;      // scene units / s at full stick
  const TURN_SPEED = 1.7;      // rad / s at full stick

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);

  const hemi = new THREE.HemisphereLight(0xffffff, 0x8e8e93, 0.9);
  scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xffffff, 2.4);
  sun.position.set(5, 9, 4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = sun.shadow.camera.bottom = -10;
  sun.shadow.camera.right = sun.shadow.camera.top = 10;
  sun.shadow.camera.far = 30;
  scene.add(sun);

  const groundMat = new THREE.MeshStandardMaterial({ color: 0xededf2, roughness: 1 });
  const ground = new THREE.Mesh(new THREE.CircleGeometry(9.5, 72), groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  function buildGrid(theme) {
    const dark = theme === "dark";
    const helper = new THREE.PolarGridHelper(9.5, 12, 8, 72, dark ? 0x3a3a3e : 0xc7c7cc, dark ? 0x2c2c2e : 0xdcdce1);
    helper.position.y = 0.002;
    return helper;
  }
  let grid = buildGrid(app.currentTheme());
  scene.add(grid);

  /* ---------------- robot model ---------------- */
  const white = new THREE.MeshStandardMaterial({ color: 0xf2f2f7, roughness: 0.55, metalness: 0.08 });
  const lightGray = new THREE.MeshStandardMaterial({ color: 0xe5e5ea, roughness: 0.6, metalness: 0.1 });
  const darkGray = new THREE.MeshStandardMaterial({ color: 0x3a3a3c, roughness: 0.7, metalness: 0.2 });
  const accentBlue = new THREE.MeshStandardMaterial({
    color: 0x0a84ff, roughness: 0.3, emissive: 0x0a84ff, emissiveIntensity: 0.35,
  });
  const gripperYellow = new THREE.MeshStandardMaterial({ color: 0xffd60a, roughness: 0.5 });

  function box(w, h, d, mat) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
    mesh.castShadow = true;
    return mesh;
  }

  const robot = new THREE.Group();
  scene.add(robot);

  // chassis: hexagonal omni base
  const chassis = new THREE.Mesh(new THREE.CylinderGeometry(1.02, 1.12, 0.32, 6), white);
  chassis.position.y = 0.36;
  chassis.castShadow = true;
  robot.add(chassis);
  const topPlate = new THREE.Mesh(new THREE.CylinderGeometry(0.8, 0.8, 0.07, 6), lightGray);
  topPlate.position.y = 0.55;
  topPlate.castShadow = true;
  robot.add(topPlate);
  // front direction marker (+Z is the robot's nose)
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.09, 0.22, 20), accentBlue);
  nose.rotation.x = Math.PI / 2;
  nose.position.set(0, 0.38, 1.08);
  robot.add(nose);

  // three kiwi wheels: ID1 front (0°), ID2 rear-left (120°), ID3 rear-right (240°)
  const wheels = [];
  for (const deg of [0, 120, 240]) {
    const angle = THREE.MathUtils.degToRad(deg);
    const pivot = new THREE.Group();
    pivot.position.set(Math.sin(angle) * 0.82, 0.21, Math.cos(angle) * 0.82);
    pivot.rotation.y = angle;            // axle radial -> rolls tangentially
    const spin = new THREE.Group();      // rotates around the axle (local Z)
    const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.21, 0.21, 0.12, 28), darkGray);
    wheel.rotation.x = Math.PI / 2;      // cylinder axis onto local Z
    wheel.castShadow = true;
    spin.add(wheel);
    const hub = box(0.05, 0.3, 0.13, accentBlue); // marker so the spin is visible
    spin.add(hub);
    pivot.add(spin);
    robot.add(pivot);
    wheels.push(spin);
  }

  // lift column (fixed) + elevating platform group
  // telescoping design: a slimmer inner pillar rides with the platform and
  // slides inside the outer column, so the lift never shows an air gap
  const column = box(0.42, 1.15, 0.42, lightGray);
  column.position.y = 1.1;                 // spans y 0.525..1.675
  robot.add(column);
  const liftGroup = new THREE.Group();
  robot.add(liftGroup);

  const innerPillar = box(0.3, 1.35, 0.3, white);
  innerPillar.position.y = -0.72;          // hidden inside the column at rest
  liftGroup.add(innerPillar);

  const platform = box(1.55, 0.1, 0.95, white);
  liftGroup.add(platform);
  // head: neck + dark display housing + blue screen
  const neck = box(0.16, 0.42, 0.16, lightGray);
  neck.position.y = 0.26;
  liftGroup.add(neck);
  const head = box(0.58, 0.34, 0.3, darkGray);
  head.position.y = 0.62;
  liftGroup.add(head);
  const screen = box(0.46, 0.24, 0.02, accentBlue);
  screen.position.set(0, 0.62, 0.16);
  liftGroup.add(screen);
  // dual arms with yellow grippers (static display pose)
  // side +1 is the robot's left (facing +Z, left = +X), side -1 is its right
  const arms = {};
  for (const side of [-1, 1]) {
    const arm = new THREE.Group();
    arm.position.set(side * 0.82, 0.02, 0);
    arm.add(box(0.16, 0.18, 0.24, white)); // shoulder
    const upper = box(0.12, 0.12, 0.42, white);
    upper.position.set(side * 0.06, -0.1, 0.2);
    upper.rotation.x = 0.5;
    arm.add(upper);
    const forearm = box(0.09, 0.09, 0.36, lightGray);
    forearm.position.set(side * 0.12, -0.22, 0.44);
    forearm.rotation.x = -0.35;
    arm.add(forearm);
    const clawTop = box(0.03, 0.1, 0.14, gripperYellow);
    clawTop.position.set(side * 0.12 + 0.03, -0.28, 0.64);
    const clawBottom = box(0.03, 0.1, 0.14, gripperYellow);
    clawBottom.position.set(side * 0.12 - 0.03, -0.28, 0.64);
    arm.add(clawTop, clawBottom);
    liftGroup.add(arm);
    arms[side] = arm;
  }

  /* ------------- one-click task props: charging pile + car -------------
   * Visible only while the one-click task runs. Positions match the backend
   * choreography at sim_move 0.22 (MOVE_SPEED 2.4 u/s): the pile stands 1.2 u
   * beyond the 3.5 s stop point, the car parks parallel on the robot's left
   * (+Z) side of the 11 s stop point — the arm action opens to the left. */
  const pileStop = 3.5 * MOVE_SPEED * 0.22;          // robot stop distance 1.85
  const carStop = 11.0 * MOVE_SPEED * 0.22;          // robot stop distance 5.81

  const taskProps = new THREE.Group();
  taskProps.visible = false;
  scene.add(taskProps);

  // charging pile: white column + blue screen + holstered gun + cable loop
  const pile = new THREE.Group();
  pile.position.set(0, 0, pileStop + 1.2);
  const pileBody = box(0.5, 1.7, 0.35, white);
  pileBody.position.y = 0.85;
  pile.add(pileBody);
  const pileBase = box(0.7, 0.12, 0.55, lightGray);
  pileBase.position.y = 0.06;
  pile.add(pileBase);
  const pileScreen = box(0.3, 0.22, 0.03, accentBlue);
  pileScreen.position.set(0, 1.28, -0.19);           // faces the arriving robot (-Z)
  pile.add(pileScreen);
  const pileGun = box(0.1, 0.24, 0.12, darkGray);
  pileGun.position.set(0.31, 0.95, 0);               // robot's left (+X) side
  pile.add(pileGun);
  const pileCable = new THREE.Mesh(new THREE.TorusGeometry(0.16, 0.025, 10, 32), darkGray);
  pileCable.position.set(0.31, 0.66, 0);
  pileCable.castShadow = true;
  pile.add(pileCable);
  taskProps.add(pile);

  // car: body + cabin + four wheels, length along X, charge port toward the path
  const carPaint = new THREE.MeshStandardMaterial({ color: 0x0a84ff, roughness: 0.35, metalness: 0.25 });
  const carGlass = new THREE.MeshStandardMaterial({ color: 0x1c1c1e, roughness: 0.15, metalness: 0.6 });
  const car = new THREE.Group();
  car.position.set(-carStop, 0, pileStop + 1.7);     // left of the robot's stop point
  const carBody = box(3.1, 0.55, 1.5, carPaint);
  carBody.position.y = 0.58;
  car.add(carBody);
  const carCabin = box(1.6, 0.48, 1.34, carGlass);
  carCabin.position.set(-0.25, 1.08, 0);
  car.add(carCabin);
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const wheelM = new THREE.Mesh(new THREE.CylinderGeometry(0.32, 0.32, 0.22, 24), darkGray);
      wheelM.rotation.x = Math.PI / 2;               // axle along Z
      wheelM.position.set(sx * 1.05, 0.32, sz * 0.78);
      wheelM.castShadow = true;
      car.add(wheelM);
    }
  }
  const chargePort = box(0.26, 0.18, 0.03, accentBlue);
  chargePort.position.set(0.9, 0.62, -0.76);         // on the side facing the robot
  car.add(chargePort);
  taskProps.add(car);

  function taskStart() {
    robot.position.set(0, 0, 0);
    yaw = 0;
    liftNorm = 0;                        // 升降台也回零
    robot.rotation.y = 0;
    liftGroup.position.y = LIFT_REST;
    taskProps.visible = true;
  }
  function taskStop() {
    taskProps.visible = false;
  }
  window.__robotTask = { start: taskStart, stop: taskStop };

  /* ---------------- theme ---------------- */
  function applyTheme(theme) {
    const dark = theme === "dark";
    groundMat.color.set(dark ? 0x0c0c0e : 0xededf2);
    const next = buildGrid(theme);
    scene.remove(grid);
    grid.geometry.dispose();
    grid.material.dispose();
    grid = next;
    scene.add(grid);
    hemi.intensity = dark ? 0.55 : 0.9;
    sun.intensity = dark ? 1.8 : 2.4;
  }

  /* ------------- spherical drag orbit (skill fallback pattern) ------------- */
  const target = new THREE.Vector3(0, 1.1, 0);
  let radius = 6.4;
  let theta = 0.70;
  let phi = 1.15;
  let dragging = false;
  let prevX = 0, prevY = 0;

  function updateCamera() {
    camera.position.set(
      target.x + radius * Math.sin(phi) * Math.sin(theta),
      target.y + radius * Math.cos(phi),
      target.z + radius * Math.sin(phi) * Math.cos(theta)
    );
    camera.lookAt(target);
  }

  renderer.domElement.addEventListener("pointerdown", (e) => {
    dragging = true;
    prevX = e.clientX;
    prevY = e.clientY;
    renderer.domElement.setPointerCapture(e.pointerId);
  });
  renderer.domElement.addEventListener("pointerup", () => { dragging = false; });
  renderer.domElement.addEventListener("pointercancel", () => { dragging = false; });
  renderer.domElement.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    theta -= (e.clientX - prevX) * 0.005;
    phi = Math.max(0.15, Math.min(1.45, phi - (e.clientY - prevY) * 0.005));
    prevX = e.clientX;
    prevY = e.clientY;
  });
  renderer.domElement.addEventListener("wheel", (e) => {
    e.preventDefault();
    radius = Math.max(2.5, Math.min(14, radius * (1 + e.deltaY * 0.001)));
  }, { passive: false });

  /* -------- onboard camera views (simulated wrist/chest cams) --------
   * Each open floating window gets a small renderer; the virtual camera is
   * mounted on the corresponding robot part so the view follows motion.
   * Exposed to index.html as window.__robotCams.attach/detach(id, canvas). */
  const camViews = new Map();
  const camDefs = {
    chest:   { parent: robot,    pos: [0, 1.45, 0.26],        tilt: -0.14 },
    wrist_l: { parent: arms[1],  pos: [0.12, -0.17, 0.46],    tilt: -0.30 },
    wrist_r: { parent: arms[-1], pos: [-0.12, -0.17, 0.46],   tilt: -0.30 },
  };

  function attachCam(id, canvas) {
    if (camViews.has(id) || !camDefs[id]) return;
    const def = camDefs[id];
    const mount = new THREE.Group();
    mount.position.set(def.pos[0], def.pos[1], def.pos[2]);
    def.parent.add(mount);
    const cam = new THREE.PerspectiveCamera(70, 16 / 9, 0.05, 60);
    mount.add(cam);
    // Aim along the parent's +Z (robot forward) with a downward pitch,
    // resolved via world-space lookAt so the orientation is unambiguous.
    // The local rotation stays fixed afterwards, so the view follows the robot.
    def.parent.updateWorldMatrix(true, false);
    mount.updateMatrixWorld(true);
    const target = new THREE.Vector3(
      def.pos[0],
      def.pos[1] + Math.sin(def.tilt),
      def.pos[2] + Math.cos(def.tilt)
    );
    def.parent.localToWorld(target);
    cam.lookAt(target);
    const r = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    camViews.set(id, { renderer: r, camera: cam, mount, canvas });
  }

  function detachCam(id) {
    const view = camViews.get(id);
    if (!view) return;
    view.mount.parent.remove(view.mount);
    view.renderer.dispose();
    camViews.delete(id);
  }

  function renderCamViews() {
    for (const view of camViews.values()) {
      const w = view.canvas.clientWidth, h = view.canvas.clientHeight;
      if (!w || !h) continue;
      const dpr = view.renderer.getPixelRatio();
      if (view.canvas.width !== Math.round(w * dpr) || view.canvas.height !== Math.round(h * dpr)) {
        view.renderer.setSize(w, h, false);
        view.camera.aspect = w / h;
        view.camera.updateProjectionMatrix();
      }
      view.renderer.render(scene, view.camera);
    }
  }

  window.__robotCams = { attach: attachCam, detach: detachCam };

  /* ---------------- animation ---------------- */
  const clock = new THREE.Clock();
  let yaw = 0;
  let liftNorm = 0;

  function animate() {
    requestAnimationFrame(animate);
    const dt = Math.min(clock.getDelta(), 0.1);
    const motion = app.getMotion();
    const tel = app.getTelemetry(); // null when disconnected or stale

    // chassis translation & rotation from control input (simulation)
    yaw += motion.omega * TURN_SPEED * dt;
    robot.rotation.y = yaw;
    const fwd = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw));
    const left = new THREE.Vector3(Math.cos(yaw), 0, -Math.sin(yaw));
    robot.position.addScaledVector(fwd, motion.vx * MOVE_SPEED * dt);
    robot.position.addScaledVector(left, motion.vy * MOVE_SPEED * dt);
    const r = Math.hypot(robot.position.x, robot.position.z);
    if (r > ARENA_RADIUS) {
      robot.position.x *= ARENA_RADIUS / r;
      robot.position.z *= ARENA_RADIUS / r;
    }

    // wheels: real speeds when telemetry is live, else spin with travel speed
    const speeds = tel ? tel.wheel_speed : null;
    const travel = Math.hypot(motion.vx, motion.vy) * 6 + Math.abs(motion.omega) * 4;
    wheels.forEach((spin, i) => {
      const rate = speeds ? speeds[i] / 120 : travel;
      spin.rotation.z += rate * dt;
    });

    // The firmware has no upper range, so this is input-driven visualization only.
    const target01 = Math.max(0, Math.min(1, liftNorm + motion.lift * 0.5 * dt));
    liftNorm += (target01 - liftNorm) * Math.min(1, dt * 10);
    liftGroup.position.y = LIFT_REST + liftNorm * LIFT_TRAVEL; // platform rides on top of the column

    // keep the camera pointed at the walking robot
    target.lerp(new THREE.Vector3(robot.position.x, 1.1 + liftNorm * 0.5, robot.position.z), Math.min(1, dt * 3));
    updateCamera();
    renderer.render(scene, camera);
    renderCamViews();
  }

  function resize() {
    const w = container.clientWidth, h = container.clientHeight;
    if (!w || !h) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  new ResizeObserver(resize).observe(container);
  resize();

  applyTheme(app.currentTheme());
  app.onThemeChange(applyTheme);
  updateCamera();
  animate();
}

console.log("robot3d module loaded, booted =", booted);
