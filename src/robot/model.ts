import {
  BufferGeometry,
  CapsuleGeometry,
  CatmullRomCurve3,
  CylinderGeometry,
  Group,
  LatheGeometry,
  Mesh,
  MeshPhysicalMaterial,
  SphereGeometry,
  TubeGeometry,
  Vector2,
  Vector3,
} from 'three'
import type { Material } from 'three'
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js'

// An original humanoid in the public design language of a modern general
// purpose robot: a glossy black head on a black hood that flares into the
// shoulders and frames a flat white chest plate, a black belly over a narrow
// waist, satin white shells on the fronts of the limbs, exposed actuators at
// the hips, forearms and calves, black hands and graphite feet. No marks or
// logos. Every bone extends along its joint's local -y; +z is forward.
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
  headY: 0.05,
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
  /** Per hand: four fingers (index to pinky) and the thumb, as [base, tip] joints. */
  fingers: [Group, Group][][]
  thumb: [Group, Group][]
  meshes: Mesh[]
  dispose: () => void
}

/**
 * Smooth shell of revolution. Profile is [radius, y], bottom to top. A sweep
 * under 1 leaves the back open, centered on +z: a shell over the front only.
 */
function lathe(profile: [number, number][], sx = 1, sz = 1, sweep = 1): BufferGeometry {
  const curve = new CatmullRomCurve3(
    profile.map(([r, y]) => new Vector3(r, y, 0)),
    false,
    'centripetal',
  )
  const pts = curve.getPoints(40).map((p) => new Vector2(Math.max(p.x, 0), p.y))
  pts[0].x = 0
  pts[pts.length - 1].x = 0
  const phi = Math.PI * 2 * sweep
  const g = new LatheGeometry(pts, Math.max(12, Math.round(48 * sweep)), -phi / 2, phi)
  g.scale(sx, 1, sz)
  return g
}

/** The same profile shrunk toward its axis, for the dark core under an open shell. */
function inset(profile: [number, number][], k: number): [number, number][] {
  return profile.map(([r, y]) => [r * k, y])
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

/** Rounded shoe: tall at the ankle, sloping down to a narrower, rounded toe. */
function shoe(w: number, h: number, d: number): BufferGeometry {
  const g = new RoundedBoxGeometry(w, h, d, 6, 0.028)
  const pos = g.attributes.position
  for (let i = 0; i < pos.count; i++) {
    const t = (pos.getZ(i) + d / 2) / d
    const y = pos.getY(i)
    if (y > -h * 0.3) pos.setY(i, -h * 0.3 + (y + h * 0.3) * (1 - 0.5 * Math.pow(t, 1.6)))
    pos.setX(i, pos.getX(i) * (1 - 0.18 * t * t))
  }
  g.computeVertexNormals()
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
const HEAD_SX = 0.9
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
    color: 0xe7e7e4,
    roughness: 0.36,
    metalness: 0.06,
    clearcoat: 0.45,
    clearcoatRoughness: 0.3,
  })
  const dark = new MeshPhysicalMaterial({
    color: 0x121214,
    roughness: 0.52,
    metalness: 0.25,
    clearcoat: 0.12,
    clearcoatRoughness: 0.5,
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
  const graphite = new MeshPhysicalMaterial({
    color: 0x3a3b3f,
    roughness: 0.55,
    metalness: 0.2,
    clearcoat: 0.2,
  })
  const seamMat = new MeshPhysicalMaterial({ color: 0x4a4b50, roughness: 0.25, metalness: 0.9 })

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

  // Pelvis: exposed machinery, a dark frame with a cross drum.
  const root = new Group()
  add(root, taperedBox(0.2, 0.11, 0.14, 0.03, [0.7, 0.85], [1, 1]), mech, 'pelvis', 0, -0.005, -0.005)
  add(root, taperedBox(0.16, 0.05, 0.12, 0.018, [1, 1], [1, 1]), dark, 'pelvis', 0, 0.06, 0)
  for (const s of [1, -1]) {
    // Hip yaw actuators sit above each leg.
    add(root, new CylinderGeometry(0.04, 0.042, 0.06, 28), mech, 'pelvis', s * DIM.hipX, 0.012, 0)
    add(root, new CylinderGeometry(0.043, 0.043, 0.012, 28), dark, 'pelvis', s * DIM.hipX, 0.045, 0)
  }

  // Torso: black core showing as a yoke and side straps around a flat white
  // chest plate, a black belly below it, and a narrow waist.
  const spine = joint(root, 0, DIM.spineY, 0)
  add(spine, taperedBox(0.18, 0.13, 0.13, 0.04, [0.9, 0.95], [1.1, 1.05]), dark, 'torso', 0, 0.04, 0)
  add(
    spine,
    lathe([
      [0, 0.062],
      [0.05, 0.066],
      [0.078, 0.088],
      [0.09, 0.12],
      [0.093, 0.16],
      [0.086, 0.18],
      [0, 0.185],
    ], 1.5, 0.98),
    dark,
    'torso',
  )
  add(spine, taperedBox(0.32, 0.29, 0.18, 0.05, [0.84, 0.95], [1, 1]), dark, 'torso', 0, 0.288, -0.004)
  add(spine, taperedBox(0.3, 0.255, 0.19, 0.045, [0.84, 0.96], [1, 1]), shell, 'torso', 0, 0.273, 0.012)
  // The hood flares from the neck into the shoulders.
  add(
    spine,
    lathe([
      [0, 0.392],
      [0.155, 0.398],
      [0.13, 0.416],
      [0.088, 0.442],
      [0.06, 0.474],
      [0.05, 0.51],
      [0, 0.52],
    ], 1, 0.56),
    dark,
    'torso',
    0,
    0,
    -0.006,
  )

  // Neck and head: a black glossy egg with a fine seam around the face.
  const neck = joint(spine, 0, DIM.neckY, 0)
  add(neck, new CylinderGeometry(0.044, 0.048, 0.13, 24), dark, 'head', 0, 0.045, -0.006)
  const head = joint(neck, 0, DIM.headY, 0)
  add(head, lathe(HEAD_PROFILE, HEAD_SX, HEAD_SZ), visor, 'head', 0, 0.112, 0.004)
  const seam = add(head, faceSeam(), seamMat, 'head', 0, 0.112, 0.004)
  seam.castShadow = false

  // Arms
  const shoulder: Group[] = []
  const elbow: Group[] = []
  const wrist: Group[] = []
  const fingers: [Group, Group][][] = []
  const thumbs: [Group, Group][] = []
  for (const s of [1, -1]) {
    const hand: [Group, Group][] = []
    const sh = joint(spine, s * DIM.shoulderX, DIM.shoulderY, 0)
    sh.rotation.order = 'ZXY'
    // The upper-arm shell rounds over the shoulder like a cap.
    add(
      sh,
      lathe([
        [0, -0.262],
        [0.04, -0.256],
        [0.046, -0.23],
        [0.053, -0.17],
        [0.059, -0.08],
        [0.061, -0.01],
        [0.056, 0.026],
        [0.036, 0.048],
        [0, 0.054],
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
    // Dark forearm: a ribbed pad below the elbow, an actuator along the
    // inside, and a white shell down the outside.
    const fore: [number, number][] = [
      [0, -0.232],
      [0.026, -0.226],
      [0.029, -0.2],
      [0.035, -0.13],
      [0.04, -0.07],
      [0.038, -0.04],
      [0, -0.032],
    ]
    add(el, lathe(inset(fore, 0.92), 1, 0.9), dark, 'arm', 0, 0, -0.004)
    const strip: [number, number][] = [
      [0, -0.236],
      [0.028, -0.23],
      [0.031, -0.2],
      [0.037, -0.13],
      [0.039, -0.108],
      [0, -0.1],
    ]
    const outer = add(el, lathe(strip, 1, 0.9, 0.5), shell, 'arm', 0, 0, -0.004)
    outer.rotation.y = (s * Math.PI) / 2
    for (let k = 0; k < 4; k++) {
      add(el, new RoundedBoxGeometry(0.056, 0.016, 0.02, 2, 0.006), dark, 'arm', 0, -0.045 - k * 0.019, 0.032)
    }
    add(el, new CapsuleGeometry(0.013, 0.1, 4, 12), mech, 'arm', -s * 0.02, -0.15, 0.022)
    const wr = joint(el, 0, -DIM.forearm, 0)
    wr.scale.setScalar(1.12)
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
      hand.push([base, tip])
    }
    const thumb = joint(wr, -s * 0.01, -0.032, 0.038)
    thumb.rotation.set(-0.55, 0, -s * 0.35)
    add(thumb, new CapsuleGeometry(0.0095, 0.024, 4, 10), mech, 'arm', 0, -0.018, 0)
    const thumbTip = joint(thumb, 0, -0.036, 0)
    thumbTip.rotation.z = -s * 0.3
    add(thumbTip, new CapsuleGeometry(0.0088, 0.016, 4, 10), mech, 'arm', 0, -0.012, 0)
    fingers.push(hand)
    thumbs.push([thumb, thumbTip])
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
    // Exposed hip actuators: pitch across the joint, roll facing forward.
    hinge(hp, 0.05, 0.08, mech, 'leg', 0, -0.006)
    const roll = add(hp, new CylinderGeometry(0.047, 0.047, 0.05, 32), mech, 'leg', s * 0.004, 0.006, 0.03)
    roll.rotation.x = Math.PI / 2
    const cap = add(hp, new CylinderGeometry(0.036, 0.036, 0.008, 32), dark, 'leg', s * 0.004, 0.006, 0.058)
    cap.rotation.x = Math.PI / 2
    // White shell over the front of the thigh, dark structure behind.
    const thigh: [number, number][] = [
      [0, -0.372],
      [0.043, -0.366],
      [0.05, -0.33],
      [0.063, -0.24],
      [0.074, -0.15],
      [0.077, -0.11],
      [0.068, -0.078],
      [0, -0.068],
    ]
    add(hp, lathe(inset(thigh, 0.9), 1, 1.02), dark, 'leg', 0, 0, 0.004)
    // The opening faces back and in, so a strip of black shows between the legs.
    add(hp, lathe(thigh, 1, 1.02, 0.78), shell, 'leg', 0, 0, 0.012).rotation.y = s * 0.5
    const kn = joint(hp, 0, -DIM.thigh, 0)
    hinge(kn, 0.044, 0.094, dark, 'leg', 0, -0.012)
    // The shin shell rises over the knee as a cap; actuators run behind the calf.
    const shin: [number, number][] = [
      [0, -0.392],
      [0.03, -0.386],
      [0.034, -0.34],
      [0.044, -0.21],
      [0.05, -0.12],
      [0.051, -0.06],
      [0.042, -0.025],
      [0, -0.012],
    ]
    add(kn, lathe(inset(shin, 0.86), 1, 1.04), dark, 'leg', 0, 0, 0.004)
    add(kn, lathe(shin, 1, 1.04, 0.7), shell, 'leg', 0, 0, 0.01).rotation.y = s * 0.45
    for (const k of [1, -1]) {
      add(kn, new CapsuleGeometry(0.017, 0.17, 4, 12), mech, 'leg', k * 0.021, -0.2, -0.03)
    }
    const an = joint(kn, 0, -DIM.shin, 0)
    hinge(an, 0.026, 0.064, dark, 'leg')
    add(an, shoe(0.09, 0.078, 0.235), graphite, 'leg', 0, -DIM.ankleH + 0.039, 0.042)
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
    fingers,
    thumb: thumbs,
    meshes,
    dispose() {
      for (const g of geometries) g.dispose()
      for (const m of [shell, dark, mech, visor, graphite, seamMat]) m.dispose()
    },
  }
}
