import {
  BufferGeometry,
  CapsuleGeometry,
  CatmullRomCurve3,
  CylinderGeometry,
  Group,
  LatheGeometry,
  Mesh,
  MeshBasicMaterial,
  MeshPhysicalMaterial,
  SphereGeometry,
  TubeGeometry,
  Vector2,
  Vector3,
} from 'three'
import type { Material } from 'three'
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js'

// An original humanoid in the public design language of a modern general
// purpose robot: black glossy head, black yoke and waist, satin white shells
// over the chest and limbs, black joints, hands and feet. No marks or logos.
// Every bone extends along its joint's local -y; +z is forward.
export const DIM = {
  hipX: 0.092,
  hipY: -0.05,
  thigh: 0.42,
  shin: 0.42,
  ankleH: 0.075,
  spineY: 0.08,
  shoulderX: 0.205,
  shoulderY: 0.365,
  upperArm: 0.285,
  forearm: 0.255,
  neckY: 0.415,
  headY: 0.022,
  standPelvis: 0.94,
  // Foot support rectangle relative to the ankle.
  heel: -0.075,
  toe: 0.16,
  footHalfW: 0.043,
}

export type Part = 'head' | 'torso' | 'pelvis' | 'arm' | 'leg'

export interface Rig {
  root: Group
  spine: Group
  neck: Group
  head: Group
  shoulder: [Group, Group]
  elbow: [Group, Group]
  wrist: [Group, Group]
  hip: [Group, Group]
  knee: [Group, Group]
  ankle: [Group, Group]
  meshes: Mesh[]
  glow: MeshBasicMaterial
  dispose: () => void
}

/** Smooth closed shell of revolution. Profile is [radius, y], bottom to top. */
function lathe(profile: [number, number][], sx = 1, sz = 1, samples = 40, segs = 48): BufferGeometry {
  const curve = new CatmullRomCurve3(
    profile.map(([r, y]) => new Vector3(r, y, 0)),
    false,
    'centripetal',
  )
  const pts = curve.getPoints(samples).map((p) => new Vector2(Math.max(p.x, 0), p.y))
  pts[0].x = 0
  pts[pts.length - 1].x = 0
  const g = new LatheGeometry(pts, segs)
  g.scale(sx, 1, sz)
  return g
}

/** Rounded box whose width and depth scale linearly from bottom to top. */
function taperedBox(
  w: number,
  h: number,
  d: number,
  r: number,
  bottom: [number, number],
  top: [number, number] = [1, 1],
): BufferGeometry {
  const g = new RoundedBoxGeometry(w, h, d, 6, r)
  const pos = g.attributes.position
  const nor = g.attributes.normal
  const n = new Vector3()
  for (let i = 0; i < pos.count; i++) {
    const t = (pos.getY(i) + h / 2) / h
    const sx = bottom[0] + (top[0] - bottom[0]) * t
    const sz = bottom[1] + (top[1] - bottom[1]) * t
    pos.setX(i, pos.getX(i) * sx)
    pos.setZ(i, pos.getZ(i) * sz)
    n.set(nor.getX(i) / sx, nor.getY(i), nor.getZ(i) / sz).normalize()
    nor.setXYZ(i, n.x, n.y, n.z)
  }
  return g
}

const HEAD_PROFILE: [number, number][] = [
  [0, -0.105],
  [0.042, -0.1],
  [0.066, -0.075],
  [0.08, -0.03],
  [0.086, 0.02],
  [0.083, 0.065],
  [0.068, 0.1],
  [0.04, 0.122],
  [0, 0.128],
]
const HEAD_SX = 0.94
const HEAD_SZ = 1.12

/** Thin light line framing the face: where a tilted plane meets the head. */
function faceSeam(): BufferGeometry {
  const curve = new CatmullRomCurve3(
    HEAD_PROFILE.map(([r, y]) => new Vector3(r, y, 0)),
    false,
    'centripetal',
  )
  const prof = curve.getPoints(80)
  const yBot = -0.06
  const yTop = 0.128
  const z0 = 0.055
  const side: Vector3[] = []
  for (const p of prof) {
    if (p.y < yBot) continue
    const zp = (z0 * (yTop - p.y)) / (yTop - yBot)
    const rz = p.x * HEAD_SZ
    if (rz <= zp || p.x < 1e-4) continue
    const cos = zp / rz
    const x = p.x * HEAD_SX * Math.sqrt(1 - cos * cos)
    side.push(new Vector3(x * 1.012, p.y, zp * 1.012))
  }
  const pts = [...side, new Vector3(0, yTop + 0.001, 0), ...side.slice().reverse().map((v) => new Vector3(-v.x, v.y, v.z))]
  return new TubeGeometry(new CatmullRomCurve3(pts), 160, 0.0024, 6, false)
}

export function buildRobot(): Rig {
  const shell = new MeshPhysicalMaterial({
    color: 0xd6d7d6,
    roughness: 0.3,
    metalness: 0.28,
    clearcoat: 0.6,
    clearcoatRoughness: 0.22,
  })
  const dark = new MeshPhysicalMaterial({
    color: 0x111113,
    roughness: 0.48,
    metalness: 0.3,
    clearcoat: 0.3,
    clearcoatRoughness: 0.4,
  })
  const mech = new MeshPhysicalMaterial({
    color: 0x2b2c30,
    roughness: 0.38,
    metalness: 0.7,
  })
  const visor = new MeshPhysicalMaterial({
    color: 0x050506,
    roughness: 0.07,
    metalness: 0.35,
    clearcoat: 1,
    clearcoatRoughness: 0.03,
  })
  const glow = new MeshBasicMaterial({ color: 0x9adfff })

  const meshes: Mesh[] = []
  const geometries: BufferGeometry[] = []

  function add(parent: Group, geo: BufferGeometry, mat: Material, part: Part, x = 0, y = 0, z = 0): Mesh {
    const m = new Mesh(geo, mat)
    m.position.set(x, y, z)
    m.castShadow = true
    m.receiveShadow = true
    m.userData.part = part
    parent.add(m)
    meshes.push(m)
    geometries.push(geo)
    return m
  }

  function joint(parent: Group, x = 0, y = 0, z = 0): Group {
    const g = new Group()
    g.position.set(x, y, z)
    parent.add(g)
    return g
  }

  /** Hinge drum whose axis runs along x. */
  function hinge(parent: Group, r: number, len: number, mat: Material, part: Part, y = 0, z = 0) {
    const m = add(parent, new CylinderGeometry(r, r, len, 28), mat, part, 0, y, z)
    m.rotation.z = Math.PI / 2
    return m
  }

  // Pelvis: black, narrowing toward the crotch, with hip drums.
  const root = new Group()
  add(root, taperedBox(0.25, 0.13, 0.17, 0.045, [0.62, 0.82], [1, 1]), dark, 'pelvis', 0, -0.005, 0)
  add(root, taperedBox(0.17, 0.05, 0.125, 0.02, [1, 1], [1.05, 1.05]), mech, 'pelvis', 0, 0.065, 0)

  // Torso: black waist, white chest plate, black yoke across the shoulders.
  const spine = joint(root, 0, DIM.spineY, 0)
  add(spine, taperedBox(0.2, 0.17, 0.145, 0.06, [0.95, 0.95], [1.12, 1.08]), dark, 'torso', 0, 0.055, 0)
  add(spine, taperedBox(0.33, 0.25, 0.2, 0.065, [0.66, 0.8], [1, 1]), shell, 'torso', 0, 0.232, 0.01)
  add(spine, taperedBox(0.36, 0.085, 0.185, 0.03, [1, 1], [0.42, 0.72]), dark, 'torso', 0, 0.39, -0.006)
  for (const s of [1, -1]) {
    add(spine, new SphereGeometry(0.052, 32, 20).scale(1, 0.9, 1), dark, 'torso', s * (DIM.shoulderX - 0.02), DIM.shoulderY + 0.008, -0.004)
  }

  // Neck and head: a black glossy egg with a light seam framing the face.
  const neck = joint(spine, 0, DIM.neckY, 0)
  add(neck, new CylinderGeometry(0.04, 0.055, 0.07, 24), dark, 'head', 0, 0.0, -0.006)
  const head = joint(neck, 0, DIM.headY, 0)
  add(head, lathe(HEAD_PROFILE, HEAD_SX, HEAD_SZ), visor, 'head', 0, 0.112, 0.004)
  const seam = add(head, faceSeam(), glow, 'head', 0, 0.112, 0.004)
  seam.castShadow = false

  // Arms
  const shoulder: Group[] = []
  const elbow: Group[] = []
  const wrist: Group[] = []
  for (const s of [1, -1]) {
    const sh = joint(spine, s * DIM.shoulderX, DIM.shoulderY, 0)
    sh.rotation.order = 'ZXY'
    add(sh, new SphereGeometry(0.05, 28, 18), dark, 'arm')
    add(
      sh,
      lathe([
        [0, -0.25],
        [0.034, -0.244],
        [0.039, -0.215],
        [0.045, -0.15],
        [0.054, -0.08],
        [0.058, -0.035],
        [0.05, 0.0],
        [0, 0.012],
      ], 1, 0.95),
      shell,
      'arm',
      0,
      0,
      0.002,
    )
    const el = joint(sh, 0, -DIM.upperArm, 0)
    hinge(el, 0.034, 0.074, dark, 'arm')
    add(el, new SphereGeometry(0.036, 20, 14), dark, 'arm', 0, -0.012, -0.004)
    add(
      el,
      lathe([
        [0, -0.232],
        [0.027, -0.226],
        [0.03, -0.2],
        [0.038, -0.13],
        [0.046, -0.07],
        [0.044, -0.04],
        [0, -0.032],
      ], 1, 0.98),
      shell,
      'arm',
    )
    const wr = joint(el, 0, -DIM.forearm, 0)
    add(wr, new CylinderGeometry(0.024, 0.027, 0.04, 20), dark, 'arm', 0, 0.006, 0)
    add(wr, new RoundedBoxGeometry(0.032, 0.088, 0.078, 3, 0.013), dark, 'arm', 0, -0.054, 0)
    for (let f = 0; f < 4; f++) {
      const len = f === 0 || f === 3 ? 0.9 : 1
      const base = joint(wr, -s * 0.003, -0.1, 0.029 - f * 0.0192)
      base.rotation.z = -s * 0.22
      add(base, new CapsuleGeometry(0.0085, 0.024 * len, 4, 10), mech, 'arm', 0, -0.016, 0)
      const tip = joint(base, 0, -0.033 * len, 0)
      tip.rotation.z = -s * 0.42
      add(tip, new CapsuleGeometry(0.008, 0.02 * len, 4, 10), mech, 'arm', 0, -0.014, 0)
    }
    const thumb = joint(wr, -s * 0.01, -0.032, 0.038)
    thumb.rotation.set(-0.55, 0, -s * 0.35)
    add(thumb, new CapsuleGeometry(0.0095, 0.024, 4, 10), mech, 'arm', 0, -0.018, 0)
    const thumbTip = joint(thumb, 0, -0.036, 0)
    thumbTip.rotation.z = -s * 0.3
    add(thumbTip, new CapsuleGeometry(0.0088, 0.016, 4, 10), mech, 'arm', 0, -0.012, 0)
    shoulder.push(sh)
    elbow.push(el)
    wrist.push(wr)
  }

  // Legs
  const hip: Group[] = []
  const knee: Group[] = []
  const ankle: Group[] = []
  for (const s of [1, -1]) {
    const hp = joint(root, s * DIM.hipX, DIM.hipY, 0)
    hinge(hp, 0.052, 0.07, mech, 'leg', 0, -0.004)
    add(hp, new SphereGeometry(0.052, 24, 16), dark, 'leg')
    add(
      hp,
      lathe([
        [0, -0.372],
        [0.042, -0.366],
        [0.048, -0.33],
        [0.06, -0.24],
        [0.073, -0.14],
        [0.079, -0.08],
        [0.074, -0.04],
        [0, -0.028],
      ], 1, 1.04),
      shell,
      'leg',
      0,
      0,
      0.006,
    )
    const kn = joint(hp, 0, -DIM.thigh, 0)
    hinge(kn, 0.046, 0.094, dark, 'leg')
    add(kn, new RoundedBoxGeometry(0.072, 0.085, 0.04, 3, 0.016), dark, 'leg', 0, -0.006, 0.03)
    add(
      kn,
      lathe([
        [0, -0.365],
        [0.032, -0.358],
        [0.037, -0.32],
        [0.046, -0.21],
        [0.052, -0.13],
        [0.05, -0.08],
        [0, -0.062],
      ], 1, 1.04),
      shell,
      'leg',
      0,
      0,
      0.004,
    )
    add(kn, new CapsuleGeometry(0.024, 0.17, 4, 12), mech, 'leg', 0, -0.2, -0.03)
    const an = joint(kn, 0, -DIM.shin, 0)
    hinge(an, 0.03, 0.07, dark, 'leg')
    add(an, new SphereGeometry(0.03, 20, 14), dark, 'leg')
    add(an, taperedBox(0.084, 0.048, 0.225, 0.02, [1, 1], [0.92, 0.9]), dark, 'leg', 0, -DIM.ankleH + 0.024, 0.042)
    hip.push(hp)
    knee.push(kn)
    ankle.push(an)
  }

  return {
    root,
    spine,
    neck,
    head,
    shoulder: shoulder as [Group, Group],
    elbow: elbow as [Group, Group],
    wrist: wrist as [Group, Group],
    hip: hip as [Group, Group],
    knee: knee as [Group, Group],
    ankle: ankle as [Group, Group],
    meshes,
    glow,
    dispose() {
      for (const g of geometries) g.dispose()
      for (const m of [shell, dark, mech, visor, glow]) m.dispose()
    },
  }
}
