// Live view of the current best policy for the lab wall.
// Unity records the policy's real rollout headlessly (tools/showcase/render.sh) and exports it;
// this page replays it in 3D in the showcase looks with cinematic camera cuts, and reloads
// when current.json points at a newer take.
//   ?look=art | minimal | cycle (default: art, static; cycle alternates every 60 s)
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';

const params = new URLSearchParams(location.search);
const LOOK_MODE = params.get('look') || 'art';   // the hall (its fins and bullseyes give the throw a scale); ?look=minimal for the studio, ?look=cycle alternates every minute
const START_AT = parseFloat(params.get('t') || '0'); // seconds into the take, for checking a moment
const LOOK_SECONDS = 60;
const POLL_SECONDS = 30;
const HIT_LINGER = 0.7;
const UP = new THREE.Vector3(0, 1, 0);

// ------------------------------------------------------------------ take

function decodePoses(b64) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return new Float32Array(bytes.buffer);
}

class Take {
  constructor(json) {
    Object.assign(this, json);
    for (const a of this.actors) {
      a.poses = decodePoses(a.poses);
      a.last = a.first + a.count - 1;
    }
    this.arm = this.actors.find(a => a.kind === 'arm');
    this.balls = this.actors.filter(a => a.kind === 'ball');
    this.targets = this.actors.filter(a => a.kind === 'target');
    this.handPart = Math.max(0, this.arm.parts.findIndex(p => p.name.toLowerCase().endsWith('palm')));
    // Reach: farthest part from the shoulder over the first seconds.
    let reach = 0;
    const base = new THREE.Vector3(), p = new THREE.Vector3();
    for (let f = 0; f < Math.min(this.arm.count, 240); f += 8) {
      this.position(this.arm, 0, f / this.hz, base);
      for (let i = 0; i < this.arm.parts.length; i++) reach = Math.max(reach, this.position(this.arm, i, f / this.hz, p).distanceTo(base));
    }
    this.armLength = Math.max(reach, 0.01);
  }
  alive(a, t) { const f = t * this.hz; return f >= a.first && f <= a.last; }
  // Pose of part i of actor a at time t, linearly interpolated between frames.
  position(a, i, t, out) {
    const n = a.parts.length;
    let f = Math.min(Math.max(t * this.hz - a.first, 0), a.count - 1);
    const f0 = Math.floor(f), f1 = Math.min(f0 + 1, a.count - 1), k = f - f0;
    const o0 = (f0 * n + i) * 7, o1 = (f1 * n + i) * 7, P = a.poses;
    return out.set(P[o0] + (P[o1] - P[o0]) * k, P[o0 + 1] + (P[o1 + 1] - P[o0 + 1]) * k, P[o0 + 2] + (P[o1 + 2] - P[o0 + 2]) * k);
  }
  rotation(a, i, t, out) {
    const n = a.parts.length;
    let f = Math.min(Math.max(t * this.hz - a.first, 0), a.count - 1);
    const f0 = Math.floor(f), f1 = Math.min(f0 + 1, a.count - 1), k = f - f0;
    const o0 = (f0 * n + i) * 7 + 3, o1 = (f1 * n + i) * 7 + 3, P = a.poses;
    out.set(P[o0], P[o0 + 1], P[o0 + 2], P[o0 + 3]);
    return out.slerp(tmpQ.set(P[o1], P[o1 + 1], P[o1 + 2], P[o1 + 3]), k);
  }
  clamp(t) { return Math.min(Math.max(t, 0), this.duration - 1 / this.hz); }
  armBase(t, out) { return this.position(this.arm, 0, this.clamp(t), out); }
  hand(t, out) { return this.position(this.arm, this.handPart, this.clamp(t), out); }
  latest(list, t, out) {
    t = this.clamp(t);
    for (let i = list.length - 1; i >= 0; i--) if (this.alive(list[i], t)) { this.position(list[i], 0, t, out); return true; }
    return false;
  }
  ball(t, out) { return this.latest(this.balls, t, out); }
  target(t, out) { return this.latest(this.targets, t, out); }
}
const tmpQ = new THREE.Quaternion();

// ------------------------------------------------------------------ camera shots (port of Shots.cs)

const SHOTS = [
  { name: 'wide',   type: 'wide',   seconds: 9, fov: 28, height: 9,  azimuth: -12, sweep: 12, distance: 1.0 },
  { name: 'close',  type: 'close',  seconds: 7, fov: 30, height: 16, azimuth: 35,  sweep: 18, distance: 1.0 },
  { name: 'orbit',  type: 'orbit',  seconds: 10, fov: 34, height: 12, azimuth: 10, sweep: 55, distance: 1.0 },
  { name: 'follow', type: 'follow', seconds: 7, fov: 34, height: 6,  azimuth: -20, sweep: 0,  distance: 1.0 },
];

const ease = u => u * u * (3 - 2 * u);
function smooth(f, t, half, taps = 9) {
  const sum = new THREE.Vector3(), v = new THREE.Vector3();
  for (let i = 0; i < taps; i++) sum.add(f(t + half * (2 * i / (taps - 1) - 1), v));
  return sum.multiplyScalar(1 / taps);
}

// Shots are locked by default (as in Shots.cs): one pose per shot, framed on positions averaged over the shot, so the
// background never drifts. shot.move = true restores the sweep and the framing that follows the action.
function evaluateShot(take, shot, t, u, start, end) {
  const L = take.armLength;
  let wArm = 0.6, wHand = 0.25, wBall = 0.35, wTarget = 0.8;
  if (!shot.move && end > start) { t = 0.5 * (start + end); u = 0.5; wArm = wHand = wBall = wTarget = Math.max(0.6, 0.5 * (end - start)); }
  const armBase = smooth((x, o) => take.armBase(x, o), t, wArm, 17);
  const hand = smooth((x, o) => take.hand(x, o), t, wHand, 17);
  // Live shots are not anchored to a hit, and targets respawn every few seconds: follow the current one,
  // smoothed so the camera pans to a new target instead of cutting.
  const fallback = take.hand(t, new THREE.Vector3());
  const target = smooth((x, o) => take.target(x, o) ? o : o.copy(fallback), t, wTarget, 17);

  const throwDir = target.clone().sub(armBase); throwDir.y = 0;
  if (throwDir.lengthSq() < 1e-6) throwDir.set(1, 0, 0);
  throwDir.normalize();
  const side = new THREE.Vector3().crossVectors(throwDir, UP).normalize(); // the camera side of the throw
  const az = THREE.MathUtils.degToRad(shot.azimuth + (shot.move ? shot.sweep * (ease(u) - 0.5) : 0));
  const h = THREE.MathUtils.degToRad(shot.height);
  const horizontal = side.clone().applyAxisAngle(UP, az);
  const viewDir = horizontal.multiplyScalar(Math.cos(h)).addScaledVector(UP, Math.sin(h)).normalize();

  let lookAt, dist;
  switch (shot.type) {
    case 'close': lookAt = hand; dist = 1.1 * L; break;
    case 'orbit': lookAt = armBase.clone().lerp(hand, 0.6); dist = 2.4 * L; break;
    case 'follow': {
      const subject = smooth((x, o) => take.ball(x, o) ? o : take.hand(x, o), t, wBall, 17);
      lookAt = subject.lerp(target, 0.25); dist = 2.2 * L; break;
    }
    default: {
      lookAt = armBase.clone().lerp(target, 0.5).addScaledVector(UP, 0.25 * L);
      const span = armBase.distanceTo(target) + 1.1 * L;
      dist = 0.5 * span / Math.tan(0.5 * THREE.MathUtils.degToRad(shot.fov)) * 1.05;
    }
  }
  dist *= shot.distance;
  return { position: lookAt.clone().addScaledVector(viewDir, dist), lookAt, fov: shot.fov };
}

// ------------------------------------------------------------------ looks

const hex = (h) => new THREE.Color(h);
function lit(color, o = {}) {
  return new THREE.MeshPhysicalMaterial({
    color: hex(color), roughness: 1 - (o.smoothness ?? 0.5), metalness: o.metallic ?? 0,
    clearcoat: o.clearcoat ?? 0, clearcoatRoughness: 1 - (o.clearcoatSmoothness ?? 0.9),
    emissive: o.emission ? hex(o.emission) : hex('#000000'), emissiveIntensity: o.emissionIntensity ?? 1,
    transparent: o.opacity !== undefined, opacity: o.opacity ?? 1, depthWrite: o.opacity === undefined,
  });
}

const LOOKS = {
  minimal: {
    label: 'Studio',
    background: '#3a3835', exposure: 1.0, bloom: { strength: 0.22, radius: 0.5, threshold: 0.9 },
    materials: () => ({
      arm: lit('#e9e6e0', { smoothness: 0.45, clearcoat: 0.8, clearcoatSmoothness: 0.92 }),
      hand: lit('#3c4048', { smoothness: 0.62, metallic: 0.45, clearcoat: 0.3 }),
      ball: lit('#ff5a14', { smoothness: 0.38 }),
      target: lit('#2fc6d2', { smoothness: 0.95, opacity: 0.24, emission: '#19c4cf', emissionIntensity: 0.3 }),
      hit: lit('#9ff4fa', { smoothness: 0.95, opacity: 0.4, emission: '#4fd8e4', emissionIntensity: 1.3 }),
      joint: null,
    }),
    bullseye: false,
    build: buildMinimalStage,
  },
  art: {
    label: 'Hall',
    background: '#171b22', exposure: 1.25, bloom: { strength: 0.55, radius: 0.6, threshold: 0.75 },
    materials: () => ({
      arm: lit('#ece8e1', { smoothness: 0.5, clearcoat: 0.9, clearcoatSmoothness: 0.95 }),
      hand: lit('#8a9099', { smoothness: 0.62, metallic: 0.85 }),
      joint: lit('#26282c', { smoothness: 0.72, metallic: 0.8 }),
      ball: lit('#1fc3dc', { smoothness: 0.55, emission: '#1fc3dc', emissionIntensity: 0.18 }),
      target: lit('#d93a2b', { smoothness: 0.55, emission: '#d93a2b', emissionIntensity: 0.45, clearcoat: 0.6 }),
      targetWhite: lit('#eee9df', { smoothness: 0.55, emission: '#eee9df', emissionIntensity: 0.12, clearcoat: 0.6 }),
      rim: lit('#2a2c30', { smoothness: 0.7, metallic: 0.8 }),
      hit: lit('#ffe2a8', { smoothness: 0.9, opacity: 0.55, emission: '#ffc468', emissionIntensity: 1.7 }),
    }),
    bullseye: true,
    build: buildArtStage,
  },
};

// ------------------------------------------------------------------ stages (both authored in arm lengths, origin on the floor under the shoulder)

function lathe(points, segments) {
  return new THREE.LatheGeometry(points.map(([r, y]) => new THREE.Vector2(r, y)), segments);
}

function buildMinimalStage(group, L, shoulderHeight, lights) {
  // Round cyclorama: floor curving into the wall all around.
  const pts = [];
  const floor = 0.62, curve = 0.38, wall = 1.4;
  for (let i = 0; i <= 12; i++) pts.push([floor * i / 12, 0]);
  for (let i = 1; i <= 24; i++) { const a = 0.5 * Math.PI * i / 24; pts.push([floor + Math.sin(a) * curve, curve - Math.cos(a) * curve]); }
  pts.push([floor + curve, wall]);
  const cove = new THREE.Mesh(lathe(pts, 128), new THREE.MeshStandardMaterial({ color: hex('#75716c'), roughness: 0.88, side: THREE.BackSide }));
  cove.scale.setScalar(20 * L);
  cove.receiveShadow = true;
  group.add(cove);
  const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.11 * L, 0.11 * L, shoulderHeight - 0.04 * L, 64),
    lit('#3a3c40', { smoothness: 0.42, metallic: 0.2 }));
  pedestal.position.y = 0.5 * (shoulderHeight - 0.04 * L);
  pedestal.castShadow = pedestal.receiveShadow = true;
  group.add(pedestal);

  const key = sun('#fff1de', 3.0, 42, -35, L, true); lights.push(key);
  lights.push(sun('#cfe0ff', 1.3, 22, 150, L, false));
  lights.push(sun('#e8eef5', 0.2, 8, -120, L, false));
  lights.push(new THREE.HemisphereLight(hex('#5a606a'), hex('#2c2a28'), 0.9));
}

function buildArtStage(group, L, shoulderHeight, lights, scene) {
  scene.fog = new THREE.FogExp2(hex('#171b22'), 0.095 / L);
  const floor = new THREE.Mesh(new THREE.CircleGeometry(60 * L, 96), lit('#16171a', { smoothness: 0.55 }));
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  group.add(floor);

  const finMat = lit('#2c2e33', { smoothness: 0.45, metallic: 0.3 });
  const strip = new THREE.MeshBasicMaterial({ color: hex('#c9d8f5').multiplyScalar(1.1) });
  const warm = new THREE.MeshBasicMaterial({ color: hex('#ffc98a').multiplyScalar(1.5) });
  const fins = 44, ring = 7.5, height = 9;
  for (let i = 0; i < fins; i++) {
    const a = 2 * Math.PI * i / fins;
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.22 * L, height * L, 0.5 * L), finMat);
    fin.position.set(Math.sin(a) * (ring + 0.25) * L, 0.5 * height * L, Math.cos(a) * (ring + 0.25) * L);
    fin.rotation.y = a;
    group.add(fin);
    if (i % 2) continue;
    const h = i % 8 === 0 ? 6 : 4.2;
    const s = new THREE.Mesh(new THREE.BoxGeometry(0.035 * L, h * L, 0.02 * L), i % 8 === 0 ? warm : strip);
    s.position.set(Math.sin(a) * (ring - 0.02) * L, (0.25 + 0.5 * h) * L, Math.cos(a) * (ring - 0.02) * L);
    s.rotation.y = a;
    group.add(s);
  }
  const mark = new THREE.Mesh(new THREE.TorusGeometry(0.75 * L, 0.006 * L, 8, 160), new THREE.MeshBasicMaterial({ color: hex('#ffb766').multiplyScalar(1.6) }));
  mark.rotation.x = Math.PI / 2;
  group.add(mark);

  const r = 0.12 * L, top = shoulderHeight - 0.04 * L;
  const plinth = new THREE.Mesh(lathe([[1.6 * r, 0], [1.6 * r, 0.012 * top], [r, 0.024 * top], [r, 0.97 * top], [1.24 * r, 0.97 * top], [1.24 * r, top], [0, top]], 96),
    lit('#2a2c30', { smoothness: 0.58, metallic: 0.65 }));
  plinth.castShadow = plinth.receiveShadow = true;
  group.add(plinth);

  const key = spot('#ffe6c4', 40, [-1.6, 4.2, 2.2], [0.7, 0.7, 0], 68, L, true); lights.push(key);
  lights.push(spot('#9ec6ff', 26, [2.2, 1.4, -3.2], [0.2, 0.8, 0], 45, L, true));
  lights.push(sun('#c8d6f0', 0.35, 80, 20, L, false));
  lights.push(new THREE.HemisphereLight(hex('#262b34'), hex('#111113'), 1.0));
}

function sun(color, intensity, pitch, yaw, L, shadows) {
  const light = new THREE.DirectionalLight(hex(color), intensity);
  // Unity's forward for Euler(pitch, yaw), mirrored in z; the light sits opposite the way it shines.
  const p = THREE.MathUtils.degToRad(pitch), y = THREE.MathUtils.degToRad(yaw);
  const forward = new THREE.Vector3(Math.sin(y) * Math.cos(p), -Math.sin(p), -Math.cos(y) * Math.cos(p));
  light.userData.dir = forward.multiplyScalar(-1);
  if (shadows) {
    light.castShadow = true;
    light.shadow.mapSize.set(2048, 2048);
    const c = light.shadow.camera; c.left = c.bottom = -2.5 * L; c.right = c.top = 2.5 * L; c.near = 0.1 * L; c.far = 12 * L;
    light.shadow.bias = -0.0004; light.shadow.normalBias = 0.02 * L; light.shadow.radius = 6;
  }
  return light;
}

function spot(color, intensity, pos, aim, angle, L, shadows) {
  // Intensity is the illuminance at one arm length, as in the Unity stage (inverse-square falloff).
  const light = new THREE.SpotLight(hex(color), intensity, 12 * L, THREE.MathUtils.degToRad(angle / 2), 0.45, 2);
  light.intensity = intensity * L * L;
  light.userData.local = new THREE.Vector3(...pos).multiplyScalar(L);
  light.userData.aim = new THREE.Vector3(...aim).multiplyScalar(L);
  if (shadows) {
    light.castShadow = true;
    light.shadow.mapSize.set(2048, 2048);
    light.shadow.bias = -0.0004; light.shadow.normalBias = 0.02 * L; light.shadow.radius = 5;
  }
  return light;
}

// ------------------------------------------------------------------ puppets

function boxRounding(e) {
  const s = [...e].sort((a, b) => a - b);
  return Math.min(0.92 * s[0], 0.22 * s[1]);
}

function partGeometry(part) {
  const [ex, ey, ez] = part.extents;
  switch (part.shape) {
    case 'ellipsoid': {
      const g = new THREE.SphereGeometry(1, 64, 32); g.scale(ex, ey, ez); return g;
    }
    case 'capsule': {
      const e = part.extents, ax = part.axis, i = (ax + 1) % 3, j = (ax + 2) % 3;
      const r = Math.min(e[ax], 0.5 * (e[i] + e[j]));
      const g = new THREE.CapsuleGeometry(r, 2 * (e[ax] - r), 16, 48); // along Y
      if (ax === 0) g.rotateZ(-Math.PI / 2); else if (ax === 2) g.rotateX(Math.PI / 2);
      return g;
    }
    default: {
      const r = boxRounding(part.extents);
      return new RoundedBoxGeometry(2 * ex, 2 * ey, 2 * ez, 5, r);
    }
  }
}

class Puppets {
  constructor(take, look, scene) {
    this.take = take; this.look = look;
    this.root = new THREE.Group(); scene.add(this.root);
    this.mats = look.materials();
    this.items = take.actors.map(a => this.build(a));
  }
  build(a) {
    const go = new THREE.Group(); this.root.add(go);
    const parts = a.parts.map(p => {
      const g = new THREE.Group(); go.add(g);
      const m = new THREE.Mesh(partGeometry(p), this.mats[p.slot] ?? this.mats.arm);
      m.position.fromArray(p.center);
      m.castShadow = m.receiveShadow = true;
      g.add(m); g.userData.mesh = m;
      return g;
    });
    const item = { a, go, parts, glow: [], shells: [] };
    if (a.kind === 'arm' && this.mats.joint) this.addJoints(item);
    if (a.kind === 'target') {
      const p = a.parts[0], radius = p.extents[0], holder = parts[0];
      if (this.look.bullseye) {
        holder.userData.mesh.visible = false;
        const face = new THREE.Group(); holder.add(face); item.face = face;
        const rings = 5;
        for (let k = 0; k < rings; k++) {
          const mat = (k % 2 ? this.mats.targetWhite : this.mats.target).clone();
          const ring = new THREE.Mesh(k === 0 ? new THREE.CircleGeometry(radius / rings, 96) : new THREE.RingGeometry(radius * k / rings, radius * (k + 1) / rings, 96), mat);
          ring.castShadow = true; face.add(ring); item.glow.push(ring);
        }
        const rim = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.045 * radius, 16, 160), this.mats.rim);
        face.add(rim);
        const shell = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.06 * radius, 16, 160), this.mats.hit.clone());
        shell.visible = false; face.add(shell); item.shells.push(shell);
      } else {
        const m = holder.userData.mesh; m.material = m.material.clone(); m.castShadow = false;
        item.glow.push(m);
        const shell = new THREE.Mesh(m.geometry, this.mats.hit.clone());
        shell.visible = false; holder.add(shell); item.shells.push(shell);
      }
    }
    return item;
  }
  // Joint spheres over capsule caps; one per meeting point (see Replay.cs).
  addJoints(item) {
    const t = this.take, a = item.a, shoulder = t.position(a, 0, 0, new THREE.Vector3());
    const ends = [];
    a.parts.forEach((p, i) => {
      if (p.shape !== 'capsule') return;
      const e = p.extents, ax = p.axis, r = Math.min(e[ax], 0.5 * (e[(ax + 1) % 3] + e[(ax + 2) % 3]));
      const pos = t.position(a, i, 0, new THREE.Vector3()), rot = t.rotation(a, i, 0, new THREE.Quaternion());
      const local = [-1, 1].map(s => { const v = new THREE.Vector3().fromArray(p.center); v.setComponent(ax, v.getComponent(ax) + s * (e[ax] - r)); return v; });
      const world = local.map(v => v.clone().applyQuaternion(rot).add(pos));
      const near = world[0].distanceTo(shoulder) < world[1].distanceTo(shoulder) ? 0 : 1;
      for (let k = 0; k < 2; k++) ends.push({ i, local: local[k], world: world[k], r, proximal: k === near });
    });
    for (const e of ends) {
      if (!e.proximal && ends.some(o => o.i !== e.i && o.proximal && o.world.distanceTo(e.world) < 2.5 * e.r)) continue;
      const j = new THREE.Mesh(new THREE.SphereGeometry(1.14 * e.r, 48, 24), this.mats.joint);
      j.position.copy(e.local); j.castShadow = true;
      item.parts[e.i].add(j);
    }
  }
  pose(t, camera) {
    const take = this.take;
    for (const it of this.items) {
      const a = it.a;
      const f = t * take.hz;
      const lingering = a.kind === 'target' && a.hit && f > a.last && f <= a.last + HIT_LINGER * take.hz;
      const visible = (f >= a.first && f <= a.last) || lingering;
      it.go.visible = visible;
      if (!visible) continue;
      const tt = Math.min(t, a.last / take.hz);
      it.parts.forEach((g, i) => { take.position(a, i, tt, g.position); take.rotation(a, i, tt, g.quaternion); g.scale.setScalar(1); });
      if (it.face) it.face.lookAt(camera.position);
      if (a.kind === 'target' && a.hit) this.flash(it, lingering ? (f - a.last) / take.hz / HIT_LINGER : 0, lingering);
    }
  }
  flash(it, k, lingering) {
    const glow = lingering ? Math.exp(-4 * k) : 0;
    const hit = this.mats.hit;
    for (const m of it.glow) {
      const base = m.userData.baseEmissive ??= { c: m.material.emissive.clone(), i: m.material.emissiveIntensity };
      m.material.emissive.copy(base.c).lerp(hit.emissive, glow);
      m.material.emissiveIntensity = base.i + (hit.emissiveIntensity - base.i) * glow;
    }
    if (lingering) it.parts.forEach(g => g.scale.setScalar(1 + 0.12 * Math.sin(Math.PI * Math.min(1, 2.5 * k))));
    const grow = 1 - Math.pow(1 - k, 3), fade = Math.pow(1 - k, 2);
    for (const s of it.shells) {
      s.visible = lingering;
      if (!lingering) continue;
      s.scale.setScalar(1 + 1.6 * grow);
      s.material.opacity = hit.opacity * fade * 0.6 / 0.55;
      s.material.emissiveIntensity = hit.emissiveIntensity * 0.6 * fade;
    }
  }
  dispose() { this.root.parent?.remove(this.root); }
}

// ------------------------------------------------------------------ app

const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.outputColorSpace = THREE.SRGBColorSpace;
document.body.prepend(renderer.domElement);

const camera = new THREE.PerspectiveCamera(30, innerWidth / innerHeight, 0.05, 8000);
let scene, composer, bloom, puppets, take, takeVersion, lookName, lookStarted = 0;
const hud = document.getElementById('hud'), tag = document.getElementById('tag');

function setLook(name) {
  lookName = name;
  const look = LOOKS[name];
  puppets?.dispose();
  scene = new THREE.Scene();
  scene.background = hex(look.background);
  renderer.toneMappingExposure = look.exposure;
  const L = take.armLength;
  const shoulder = take.armBase(0, new THREE.Vector3());
  const stage = new THREE.Group();
  stage.position.set(shoulder.x, take.floorY, shoulder.z);
  scene.add(stage);
  const lights = [];
  look.build(stage, L, shoulder.y - take.floorY, lights, scene);
  for (const l of lights) {
    if (l.isDirectionalLight) { l.position.copy(shoulder).addScaledVector(l.userData.dir, 6 * L); l.target.position.copy(shoulder); scene.add(l.target); }
    if (l.isSpotLight) { l.position.copy(l.userData.local).add(stage.position); l.target.position.copy(l.userData.aim).add(stage.position); scene.add(l.target); }
    scene.add(l);
  }
  puppets = new Puppets(take, look, scene);
  composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  bloom = new UnrealBloomPass(new THREE.Vector2(innerWidth, innerHeight), look.bloom.strength, look.bloom.radius, look.bloom.threshold);
  composer.addPass(bloom);
  composer.addPass(new OutputPass());
  tag.textContent = `${look.label} · live replay`;
}

async function loadCurrent() {
  const res = await fetch(`current.json?ts=${Date.now()}`, { cache: 'no-store' });
  if (!res.ok) throw new Error(`current.json: ${res.status}`);
  const cur = await res.json();
  if (cur.version === takeVersion) return false;
  const data = await (await fetch(`${cur.take}?v=${encodeURIComponent(cur.version)}`, { cache: 'no-store' })).json();
  take = new Take(data);
  takeVersion = cur.version;
  setLook(LOOK_MODE === 'cycle' ? (lookName ?? 'art') : LOOK_MODE);
  playStart = performance.now() / 1000 - START_AT;
  hud.innerHTML = `<b>${data.policy}</b> &nbsp;<span class="dim">current best policy · ${data.hits.length} hit${data.hits.length === 1 ? '' : 's'} in ${data.throws} throws, ${Math.round(data.duration)} s rollout</span>`;
  return true;
}

let playStart = 0;
function frame() {
  requestAnimationFrame(frame);
  if (!take) return;
  const now = performance.now() / 1000;
  if (LOOK_MODE === 'cycle' && now - lookStarted > LOOK_SECONDS) {
    if (lookStarted) setLook(lookName === 'art' ? 'minimal' : 'art');
    lookStarted = now;
  }
  const t = (now - playStart) % take.duration;
  // Shots cycle on the playback clock; each frames the action around it.
  const cycle = SHOTS.reduce((s, x) => s + x.seconds, 0);
  let local = t % cycle, shot = SHOTS[0];
  for (const s of SHOTS) { if (local < s.seconds) { shot = s; break; } local -= s.seconds; }
  const start = t - local, end = start + shot.seconds;
  const pose = evaluateShot(take, shot, t, local / shot.seconds, start, end);
  camera.position.copy(pose.position);
  camera.fov = pose.fov; camera.updateProjectionMatrix();
  camera.lookAt(pose.lookAt);
  puppets.pose(t, camera);
  composer.render();
}

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight); composer?.setSize(innerWidth, innerHeight);
});

loadCurrent().then(() => {
  lookStarted = performance.now() / 1000;
  frame();
  setInterval(() => loadCurrent().catch(e => console.warn(e)), POLL_SECONDS * 1000);
}).catch(e => { document.body.insertAdjacentHTML('beforeend', `<div class="err">No policy published yet (${e.message}).</div>`); });

window.__liveReady = () => !!take;
