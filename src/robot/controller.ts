import { Euler, Matrix4, Quaternion, Vector2, Vector3 } from 'three'
import { DIM } from './model'
import type { Part, Rig } from './model'
import { Gestures } from './gestures'
import {
  Spring,
  clamp,
  clampToPoly,
  convexHull,
  damp,
  fwd,
  left,
  lerpAngle,
  smoothstep,
  wrapAngle,
} from './math'

// Balance is a linear inverted pendulum (LIPM) driven through its divergent
// component of motion, the "capture point" xi = c + v/w. While xi stays inside
// the support polygon the center of pressure can pull it back; once it leaves,
// the robot steps so that xi lands inside the new support. Walking is the same
// controller with the foot placed slightly short of xi so momentum carries on.
//
// Going for the ball is planned instead: footsteps all the way to a plant
// beside the ball, then the kick. The capture point follows a reference
// computed backward from the last step (DCM planning), so the run-up flows
// without stopping.
//
// Dragging is a spring from the pointer to the grabbed point. Its sideways
// part pushes the pendulum and tips the body, pushing down crouches, and
// pulling up harder than its weight lifts it off the ground: then it swings
// as a rigid body with dangling legs until it lands again.

const G = 9.81
const COM_H = 0.98
const W = Math.sqrt(G / COM_H)
const W2 = W * W
const K_XI = 2.4
const MARGIN = 0.012
const STANCE_W = 0.2
const MIN_LAT = 0.14
const MAX_STEP = 0.72
const W_OUT = 0.05
const LEG_MAX = DIM.thigh + DIM.shin - 0.004
const FOOT_CENTER = (DIM.heel + DIM.toe) / 2
const WALK_T = 0.46
const HOME_R = 0.4
const BALL_R = 0.11

// Dragging, per unit mass.
const DRAG_K = 30
const DRAG_D = 5
const AIR_K = 90
const AIR_D = 9
const EXT_MAX = 6
/** Pelvis drop when pushed down, and rise onto the toes when pulled up. */
const SQUAT_MIN = -0.24
const SQUAT_MAX = 0.045
/** Tilting inertia about the pelvis, per unit mass (m²). */
const INERTIA = 0.14

// Ball approach.
const STEP_T = 0.4
const K_DCM = 3.5
const K_SHIFT = 5
/** Plant ankle behind the ball center, along the kick. */
const PLANT_BACK = 0.06
/** Kick-foot ankle behind the ball center at its last placement. */
const KICK_GAP = 0.34
const KICK_T = 0.72
const KICK_STRIKE = 0.52

type StepKind = 'capture' | 'walk' | 'tidy' | 'plan' | 'kick'

/** What the controller needs to know about the ball. */
export interface BallLike {
  readonly pos: Vector3
  readonly vel: Vector3
  readonly active: boolean
  readonly reachable: boolean
  readonly held: boolean
  kick(v: Vector3): void
}

interface Key {
  t: number
  x: number
  z: number
  y: number
  pitch: number
}

interface Foot {
  side: 1 | -1
  pos: Vector2
  yaw: number
  planted: boolean
  from: Vector2
  fromYaw: number
  to: Vector2
  toYaw: number
  t: number
  dur: number
  height: number
  lift: number
  /** Lift carried over when a step is retargeted mid-air. */
  lift0: number
  pitch: number
}

interface PlanStep {
  foot: number
  pos: Vector2
  yaw: number
  kind: 'plan' | 'kick'
  dur: number
}

interface Frame {
  p: Vector2
  yaw: number
}

interface Drag {
  target: Vector3
  /** Where the pull acts, in the body frame: the grabbed point, or for an arm its shoulder. */
  r: Vector3
  /** Shoulder to the grabbed point when holding an arm, else 0. */
  reach: number
  arm: number
  /** Legs are only dragged sideways; anything above the hips can be lifted. */
  canLift: boolean
  lift: number
}

interface Air {
  pos: Vector3
  vel: Vector3
  q: Quaternion
  w: Vector3
  feet: { p: Vector3; v: Vector3 }[]
  /** 0 while held, toward 1 in free fall, when the legs reach for the floor. */
  free: number
  k: number
}

function makeFoot(side: 1 | -1, x: number, z: number, yaw: number): Foot {
  return {
    side,
    pos: new Vector2(x, z),
    yaw,
    planted: true,
    from: new Vector2(x, z),
    fromYaw: yaw,
    to: new Vector2(x, z),
    toYaw: yaw,
    t: 0,
    dur: 0.4,
    height: 0.05,
    lift: 0,
    lift0: 0,
    pitch: 0,
  }
}

function bezier(p0: Vector2, p1: Vector2, p2: Vector2, p3: Vector2, s: number, out: Vector2): Vector2 {
  const u = 1 - s
  const a = u * u * u
  const b = 3 * u * u * s
  const c = 3 * u * s * s
  const d = s * s * s
  return out.set(
    a * p0.x + b * p1.x + c * p2.x + d * p3.x,
    a * p0.y + b * p1.y + c * p2.y + d * p3.y,
  )
}

const _f = new Vector2()
const _l = new Vector2()
const _a = new Vector2()
const _b = new Vector2()
const _xi = new Vector2()
const _xiE = new Vector2()
const _pDes = new Vector2()
const _out = new Vector2()
const _acc = new Vector2()
const _s = new Vector2()
const _hp = new Vector2()
const _lean = new Vector2()
const _g = new Vector2()
const _cf = new Vector2()
const _cl = new Vector2()

const _v3a = new Vector3()
const _v3b = new Vector3()
const _v3c = new Vector3()
const _v3d = new Vector3()
const _F = new Vector3()
const _tau = new Vector3()
const _H = new Vector3()
const _A = new Vector3()
const _K = new Vector3()
const _pole = new Vector3()
const _m = new Matrix4()
const _qRoot = new Quaternion()
const _qRootInv = new Quaternion()
const _qThigh = new Quaternion()
const _qShin = new Quaternion()
const _qFoot = new Quaternion()
const _qTmp = new Quaternion()
const _qX = new Quaternion()
const _q1 = new Quaternion()
const _q2 = new Quaternion()
const _e = new Euler()
const X_AXIS = new Vector3(1, 0, 0)
const Y_UP = new Vector3(0, 1, 0)

export class Controller {
  readonly home: Vector2
  readonly homeYaw: number
  readonly reduced: boolean

  c: Vector2
  v = new Vector2()
  p = new Vector2()
  ext = new Vector2()
  feet: [Foot, Foot]

  private swing = -1
  private lastSwing = 1
  private stepKind: StepKind = 'capture'
  private mode: 'stand' | 'walk' | 'plan' = 'stand'
  private walkDir = new Vector2(0, 1)
  private walkLaunched = false
  private walkSteps = 0
  private walkFace = false
  /** Whether to turn and face home on the way back; decided once per trip. */
  private tripFace: boolean | null = null
  private launchSwing = 0
  private launchTime = 0
  private walkBest = 0
  private walkStall = 0
  private heading: number
  private dsTime = 1
  private settledTime = 0
  private time = 0
  private tidyPhase: 'none' | 'shift' = 'none'
  private tidySwing = 0
  private tidyTime = 0
  private drag: Drag | null = null
  private air: Air | null = null
  /** Drag force and where it acts relative to the pelvis, per unit mass. */
  private pull = new Vector3()
  private lever = new Vector3()

  // Ball chasing: footsteps to a plant beside the ball, then the kick.
  ball: BallLike | null = null
  private chase = false
  private chaseCooldown = 0
  private plan: PlanStep[] = []
  private planGoal: 'ball' | 'home' = 'home'
  private planCount = 0
  private planCop = new Vector2()
  private planXi0 = new Vector2()
  private planShift = false
  private planWait = 0
  private replan = false
  private stepTarget = new Vector2()
  private pWant = new Vector2()
  private kickDir = new Vector2(0, 1)
  private kickBall = new Vector2()
  private kickKeys: Key[] = []
  private kickStruck = false
  private kickArc = 0

  private aFilt = new Vector2()
  private vSlow = new Vector2()

  private leanP = new Spring()
  private leanR = new Spring()
  private twist = new Spring()
  private squat = new Spring()
  private headYaw = new Spring()
  private headPitch = new Spring()
  private armP = [new Spring(0.04), new Spring(0.04)]
  private armA = [new Spring(0.1), new Spring(0.1)]
  private armE = [new Spring(-0.22), new Spring(-0.22)]
  private armAim: [number, number, number] | null = null
  private armT = [new Spring(), new Spring()]
  private armWP = [new Spring(), new Spring()]
  private headRoll = new Spring()
  private shrug = new Spring()
  private curl = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
  ]
  private palmRoll = [0, 0]
  readonly gest = new Gestures()
  /** World positions from the last pose, for gestures and the scene. */
  readonly palmW = [new Vector3(), new Vector3()]
  readonly headW = new Vector3(0, 1.6, 0)

  private pelvisH = DIM.standPelvis
  private pelvisYaw: number

  private shift = 0
  private shiftTarget = 0
  private shiftTimer = 6

  lookTarget = new Vector3(0, 1.6, 5)
  /** The camera, for glances and kick direction. */
  viewer = new Vector3(0, 1.2, 7)

  constructor(x: number, z: number, yaw: number, reduced: boolean) {
    this.home = new Vector2(x, z)
    this.homeYaw = yaw
    this.heading = yaw
    this.pelvisYaw = yaw
    this.reduced = reduced
    const l = left(yaw)
    const half = STANCE_W / 2
    this.feet = [
      makeFoot(1, x + l.x * half, z + l.y * half, yaw),
      makeFoot(-1, x - l.x * half, z - l.y * half, yaw),
    ]
    const f = fwd(yaw)
    this.c = new Vector2(x + f.x * FOOT_CENTER, z + f.y * FOOT_CENTER)
  }

  // ---------------------------------------------------------------- input

  poke(dir: Vector2, mag: number, hit: Vector3, part: Part) {
    const scale = part === 'head' ? 0.6 : part === 'leg' ? 0.8 : 1
    const dv = _a.copy(dir).normalize().multiplyScalar(mag * scale)
    if (this.air) {
      const A = this.air
      A.vel.x += dv.x
      A.vel.z += dv.y
      _v3a.set(dv.x, 0, dv.y)
      _v3b.copy(hit).sub(A.pos)
      A.w.addScaledVector(_v3c.crossVectors(_v3b, _v3a), 1 / INERTIA)
      return
    }
    this.v.add(dv)
    const yaw = this.pelvisYaw
    const F = fwd(yaw, _f)
    const L = left(yaw, _l)
    const hf = clamp((hit.y - COM_H) / 0.45, -1, 1)
    this.leanP.v += dv.dot(F) * 4.5 * hf
    this.leanR.v -= dv.dot(L) * 4.5 * hf
    const rx = hit.x - this.c.x
    const rz = hit.z - this.c.y
    this.twist.v += (rz * dv.x - rx * dv.y) * 22
    if (part === 'head') {
      this.headPitch.v += dv.dot(F) * 10
      this.headYaw.v += (rz * dv.x - rx * dv.y) * 40
    }
    if (part === 'arm') {
      for (const i of [0, 1]) {
        this.armP[i].v += dv.dot(F) * 3
        this.armA[i].v += 2.5
      }
    }
    this.interrupt()
  }

  startDrag(hit: Vector3, arm: number, part: Part) {
    const q = this.bodyQuat(_q1)
    const pos = this.bodyPos(_v3a)
    const r = new Vector3()
    let reach = 0
    if (arm >= 0) {
      r.set(this.feet[arm].side * DIM.shoulderX, DIM.spineY + DIM.shoulderY, 0)
      _v3b.copy(r).applyQuaternion(q).add(pos)
      reach = clamp(hit.distanceTo(_v3b), 0.1, DIM.upperArm + DIM.forearm + 0.1)
    } else {
      r.copy(hit).sub(pos).applyQuaternion(_q2.copy(q).invert())
    }
    // Hanging from below the hips would turn it upside down, so legs only drag.
    this.drag = { target: hit.clone(), r, reach, arm, canLift: part !== 'leg' && r.y > -0.05, lift: 0 }
    this.interrupt()
  }

  updateDrag(target: Vector3) {
    if (this.drag) this.drag.target.copy(target)
  }

  /** Let go, carrying the pointer's velocity (m/s). */
  endDrag(fling: Vector3) {
    if (this.air) this.air.vel.addScaledVector(fling, 0.8)
    else {
      this.v.x += fling.x * 0.4
      this.v.y += fling.z * 0.4
    }
    this.drag = null
    this.armAim = null
    this.interrupt()
  }

  /** A ball hit it: n points from the body toward the ball. */
  hitByBall(n: Vector3, speed: number, point: Vector3, part: Part) {
    const dir = new Vector2(-n.x, -n.z)
    const horiz = dir.length()
    if (horiz < 1e-3) fwd(this.pelvisYaw, dir).negate()
    const mag = clamp(speed * 0.085, 0, 0.5) * clamp(horiz * 1.4, 0.25, 1)
    this.poke(dir, mag, point, part)
    if (part === 'head') this.headPitch.v += clamp(n.y, 0, 1) * speed * 2.5
  }

  get chasing(): boolean {
    return this.chase
  }

  get canGesture(): boolean {
    return !this.drag && !this.chase && !this.air
  }

  /** Pelvis height, for the shadow. */
  get height(): number {
    return this.pelvisH
  }

  /** Something to do when nobody is around. */
  bored(): boolean {
    if (this.reduced || !this.canGesture || this.mode !== 'stand' || this.gest.active || this.settledTime < 1.5) return false
    this.gest.bored()
    return true
  }

  private interrupt() {
    this.mode = 'stand'
    this.plan.length = 0
    this.heading = this.homeYaw
    this.tripFace = null
    this.walkLaunched = false
    this.tidyPhase = 'none'
    this.settledTime = 0
    this.gest.cancel()
    if (this.swing >= 0 && this.stepKind !== 'capture') {
      const sw = this.feet[this.swing]
      if (this.stepKind === 'kick') {
        // Abort the kick mid-air and turn it into a catch step from here.
        sw.from.copy(sw.pos)
        sw.to.copy(sw.pos)
        sw.fromYaw = sw.toYaw = sw.yaw
        sw.lift0 = sw.lift
        sw.height = 0.04
        sw.t = 0
        sw.dur = 0.3
        this.kickStruck = true
      }
      this.stepKind = 'capture'
    }
  }

  // ------------------------------------------------------------- geometry

  private footCenter(f: { pos: Vector2; yaw: number }, out: Vector2): Vector2 {
    fwd(f.yaw, out)
    return out.multiplyScalar(FOOT_CENTER).add(f.pos)
  }

  private footCorners(f: { pos: Vector2; yaw: number }, out: Vector2[]) {
    const F = fwd(f.yaw, _f)
    const L = left(f.yaw, _l)
    const hw = DIM.footHalfW - MARGIN
    for (const [a, b] of [
      [DIM.heel + MARGIN, hw],
      [DIM.heel + MARGIN, -hw],
      [DIM.toe - MARGIN, hw],
      [DIM.toe - MARGIN, -hw],
    ]) {
      out.push(new Vector2(f.pos.x + F.x * a + L.x * b, f.pos.y + F.y * a + L.y * b))
    }
  }

  private supportPoly(): Vector2[] {
    const pts: Vector2[] = []
    for (const f of this.feet) if (f.planted) this.footCorners(f, pts)
    return convexHull(pts)
  }

  private footPoly(f: Foot): Vector2[] {
    const pts: Vector2[] = []
    this.footCorners(f, pts)
    return convexHull(pts)
  }

  private midCenters(out: Vector2): Vector2 {
    this.footCenter(this.feet[0], out)
    this.footCenter(this.feet[1], _b)
    return out.add(_b).multiplyScalar(0.5)
  }

  private mid(out: Vector2): Vector2 {
    return out.copy(this.feet[0].pos).add(this.feet[1].pos).multiplyScalar(0.5)
  }

  /** Body orientation: yaw with the lean, or the free rigid body in the air. */
  private bodyQuat(out: Quaternion): Quaternion {
    if (this.air) return out.copy(this.air.q)
    return out.setFromEuler(_e.set(this.leanP.x, this.pelvisYaw, this.leanR.x, 'YXZ'))
  }

  private bodyPos(out: Vector3): Vector3 {
    return this.air ? out.copy(this.air.pos) : out.set(this.c.x, this.pelvisH, this.c.y)
  }

  // ------------------------------------------------------------- dragging

  /** Spring force of the pointer on the body, per unit mass. Sets `lever`. */
  private dragForce(k: number, d: number, out: Vector3): Vector3 {
    const D = this.drag!
    this.lever.copy(D.r).applyQuaternion(this.bodyQuat(_q1))
    out.copy(D.target).sub(this.bodyPos(_v3a)).sub(this.lever)
    if (D.reach > 0) {
      // Holding a hand: the arm bends to follow and only pulls on the
      // shoulder once it is stretched out.
      const len = out.length()
      if (len <= D.reach) return out.set(0, 0, 0)
      out.multiplyScalar(1 - D.reach / len)
      // A straight arm hardly gives, unlike the body's own sway.
      k *= 1.6
    }
    if (!D.canLift) out.y = 0
    out.multiplyScalar(k)
    if (this.air) _v3b.crossVectors(this.air.w, this.lever).add(this.air.vel)
    else _v3b.set(this.v.x, this.squat.v, this.v.y)
    return out.addScaledVector(_v3b, -d)
  }

  private liftOff() {
    this.air = {
      pos: this.bodyPos(new Vector3()),
      vel: new Vector3(this.v.x, Math.max(this.squat.v, 0), this.v.y),
      q: this.bodyQuat(new Quaternion()),
      w: new Vector3(),
      feet: this.feet.map((f) => ({ p: new Vector3(f.pos.x, DIM.ankleH + f.lift, f.pos.y), v: new Vector3() })),
      free: 0,
      k: DRAG_K,
    }
    // The whole body tilts now, so the spine straightens.
    this.leanP.x = this.leanP.v = 0
    this.leanR.x = this.leanR.v = 0
    this.squat.x = this.squat.v = 0
    this.swing = -1
    for (const f of this.feet) {
      f.planted = false
      f.lift0 = 0
    }
    if (this.chase) this.endChase(1)
    this.interrupt()
  }

  private stepAir(h: number) {
    const A = this.air!
    const held = !!this.drag
    A.free = damp(A.free, held ? 0 : 1, 5, h)
    A.k = damp(A.k, AIR_K, 2.5, h)
    const F = _F.set(0, 0, 0)
    _tau.set(0, 0, 0)
    if (held) {
      this.dragForce(A.k, AIR_D, F)
      _tau.crossVectors(this.lever, F)
    }
    this.pull.copy(F)
    // Gravity acts at the center, so only the pull turns it. It also holds
    // itself upright, firmly once let go so it lands on its feet.
    const up = _v3a.set(0, 1, 0).applyQuaternion(A.q)
    _tau.addScaledVector(_v3b.crossVectors(up, Y_UP), held ? 4 : 30)
    A.w.addScaledVector(_tau, h / INERTIA)
    A.w.multiplyScalar(Math.exp(-h * (held ? 1.5 : 6)))
    const wl = A.w.length()
    if (wl > 1e-6) {
      _q2.setFromAxisAngle(_v3a.copy(A.w).divideScalar(wl), wl * h)
      A.q.premultiply(_q2).normalize()
    }
    A.vel.addScaledVector(F, h)
    A.vel.y -= G * h
    A.vel.multiplyScalar(Math.exp(-h * 0.15))
    A.pos.addScaledVector(A.vel, h)

    // Legs hang from the hips and trail the swing; falling, they reach down.
    _e.setFromQuaternion(A.q, 'YXZ')
    const yaw = _e.y
    const L = left(yaw, _l)
    let low = Infinity
    for (const i of [0, 1]) {
      const f = this.feet[i]
      const P = A.feet[i]
      const hip = _v3a.set(f.side * DIM.hipX, DIM.hipY, 0).applyQuaternion(A.q).add(A.pos)
      const hang = _v3b.set(0, -0.93 * LEG_MAX, 0.05).applyQuaternion(A.q)
      _v3c.set(L.x * f.side * 0.03, -0.97 * LEG_MAX, L.y * f.side * 0.03)
      hang.lerp(_v3c, A.free).add(hip)
      _v3d.copy(hang).sub(P.p).multiplyScalar(160).addScaledVector(P.v, -14)
      P.v.addScaledVector(_v3d, h)
      P.p.addScaledVector(P.v, h)
      _v3d.copy(P.p).sub(hip)
      const len = _v3d.length()
      if (len > LEG_MAX) P.p.copy(hip).addScaledVector(_v3d, LEG_MAX / len)
      if (P.p.y < DIM.ankleH) {
        P.p.y = DIM.ankleH
        if (P.v.y < 0) P.v.y = 0
      }
      f.pos.set(P.p.x, P.p.z)
      f.lift = P.p.y - DIM.ankleH
      f.yaw = yaw
      f.pitch = 0.35 - 0.3 * A.free
      low = Math.min(low, f.lift)
    }
    this.c.set(A.pos.x, A.pos.z)
    this.v.set(A.vel.x, A.vel.z)
    this.ext.set(0, 0)
    this.aFilt.lerp(_a.set(F.x, F.z), 1 - Math.exp(-h * 14))
    this.vSlow.lerp(this.v, 1 - Math.exp(-h * 3.5))
    this.pelvisYaw = yaw
    this.pelvisH = A.pos.y

    // Down once the feet reach the floor and nothing is holding it up.
    const carried = held && F.y > 0.6 * G
    if ((low < 0.004 && A.vel.y < 0.3 && !carried) || A.pos.y < 0.6) this.land()
    this.updateSprings(h)
  }

  private land() {
    const A = this.air!
    this.air = null
    _e.setFromQuaternion(A.q, 'YXZ')
    const yaw = _e.y
    this.c.set(A.pos.x, A.pos.z)
    this.v.set(A.vel.x, A.vel.z)
    if (this.v.length() > 2.5) this.v.setLength(2.5)
    this.pelvisYaw = yaw
    // Whatever tilt it lands with carries on as a lean to recover from.
    this.leanP.x = clamp(_e.x, -0.4, 0.4)
    this.leanR.x = clamp(_e.z, -0.35, 0.35)
    this.leanP.v = this.leanR.v = 0
    this.pelvisH = A.pos.y
    this.squat.x = clamp(A.pos.y - DIM.standPelvis, SQUAT_MIN, 0)
    this.squat.v = Math.min(A.vel.y, 0) * 0.8
    const lo = A.feet[0].p.y <= A.feet[1].p.y ? 0 : 1
    for (const i of [0, 1]) {
      const f = this.feet[i]
      f.pos.set(A.feet[i].p.x, A.feet[i].p.z)
      f.yaw = yaw
      f.lift = f.pitch = 0
      f.planted = true
    }
    this.swing = -1
    this.dsTime = 0
    const hi = this.feet[1 - lo]
    const hiLift = A.feet[1 - lo].p.y - DIM.ankleH
    if (hiLift > 0.015) {
      // The other foot is still up: bring it down as a catch step.
      hi.planted = false
      hi.from.copy(hi.pos)
      hi.to.copy(hi.pos)
      hi.fromYaw = hi.toYaw = yaw
      hi.t = 0
      hi.dur = 0.24
      hi.height = 0.02
      hi.lift0 = hi.lift = hiLift
      this.swing = this.lastSwing = 1 - lo
      this.stepKind = 'capture'
    }
    this.interrupt()
  }

  // ------------------------------------------------------------ simulation

  step(h: number) {
    this.time += h
    if (this.air) {
      this.refreshChase(h)
      this.stepAir(h)
      return
    }
    const { c, v } = this

    this.ext.set(0, 0)
    let fy = 0
    if (this.drag) {
      const F = this.dragForce(DRAG_K, DRAG_D, this.pull)
      this.ext.set(F.x, F.z)
      if (this.ext.length() > EXT_MAX) this.ext.setLength(EXT_MAX)
      fy = F.y
      // Up on its toes and still pulled harder than its weight: it lifts off.
      if (this.drag.canLift && fy > G && this.squat.x > SQUAT_MAX - 0.01) {
        this.drag.lift += h
        if (this.drag.lift > 0.05) {
          this.liftOff()
          return
        }
      } else this.drag.lift = 0
    } else this.pull.set(0, 0, 0)

    // The legs act as a spring under the pelvis.
    const sq = this.squat
    sq.v += (-55 * sq.x - 11 * sq.v + clamp(fy, -16, 16)) * h
    sq.x += sq.v * h
    if (sq.x < SQUAT_MIN) {
      sq.x = SQUAT_MIN
      sq.v = Math.max(sq.v, 0)
    } else if (sq.x > SQUAT_MAX) {
      sq.x = SQUAT_MAX
      sq.v = Math.min(sq.v, 0)
    }

    const xi = _xi.copy(v).divideScalar(W).add(c)
    const xiE = _xiE.copy(this.ext).divideScalar(W2).add(xi)

    let outside = 0
    let gap = 0
    const planning = this.mode === 'plan'
    if (this.swing < 0) {
      const poly = this.supportPoly()
      if (planning && this.plan.length) {
        // Bring the capture point to where the next planned step starts it.
        _pDes.copy(xi).sub(this.planXi0).multiplyScalar(K_SHIFT / W).add(xi)
      } else {
        const s = this.standTarget(h)
        _pDes.copy(xi).sub(s).multiplyScalar(K_XI / W).add(xiE)
      }
      clampToPoly(_pDes, poly, this.p)
      outside = clampToPoly(xiE, poly, _out)
      gap = outside - 0.5
    } else {
      const sw = this.feet[this.swing]
      const st = this.feet[1 - this.swing]
      const poly = this.footPoly(st)
      if (this.stepKind === 'walk') {
        this.footCenter(st, this.p)
      } else if (planning && (this.stepKind === 'plan' || this.stepKind === 'kick')) {
        // Track the plan's capture point: p = cop + (1 + k/w)(xi - xi_ref).
        const e = Math.exp(W * sw.t * sw.dur)
        _s.copy(this.planXi0).sub(this.planCop).multiplyScalar(e).add(this.planCop)
        this.pWant.copy(xi).sub(_s).multiplyScalar(1 + K_DCM / W).add(this.planCop)
        clampToPoly(this.pWant, poly, this.p)
      } else {
        const center = this.footCenter(st, _s)
        _pDes.copy(xi).sub(center).multiplyScalar(K_XI / W).add(xiE)
        clampToPoly(_pDes, poly, this.p)
      }
      gap = clampToPoly(xiE, poly, _out) - 0.62
    }

    _acc.copy(c).sub(this.p).multiplyScalar(W2).add(this.ext)
    v.addScaledVector(_acc, h)

    // Safety net: past what any step could catch, bleed off the velocity that
    // carries the capture point away from the feet, so it can never diverge.
    if (gap > 0) {
      const n = _b.copy(xiE).sub(_out).normalize()
      const away = v.dot(n)
      if (away > 0) v.addScaledVector(n, -away * (1 - Math.exp(-h * 50 * gap)))
    }
    if (v.length() > 3) v.setLength(3)
    c.addScaledVector(v, h)

    const ka = 1 - Math.exp(-h * 14)
    this.aFilt.lerp(_acc, ka)
    this.vSlow.lerp(v, 1 - Math.exp(-h * 3.5))

    this.updateSteps(h, outside, xiE)
    this.updateSprings(h)
  }

  private standTarget(h: number): Vector2 {
    if (this.tidyPhase === 'shift') return this.footCenter(this.feet[1 - this.tidySwing], _s)
    if (this.mode === 'walk' && !this.walkLaunched) {
      // Shift weight over the stance foot, leaning slightly toward the path.
      return this.footCenter(this.feet[1 - this.launchSwing], _s).addScaledVector(this.walkDir, 0.05)
    }
    const s = this.midCenters(_s)
    if (!this.reduced) {
      this.shiftTimer -= h
      if (this.shiftTimer <= 0) {
        this.shiftTimer = 5 + Math.random() * 7
        this.shiftTarget = this.settledTime > 2 && Math.random() < 0.7 ? (Math.random() * 2 - 1) * 0.045 : 0
      }
      this.shift = damp(this.shift, this.settledTime > 1 ? this.shiftTarget : 0, 1.4, h)
      const L = left(this.heading, _l)
      const F = fwd(this.heading, _f)
      const t = this.time
      s.addScaledVector(L, this.shift + Math.sin(t * 0.43) * 0.006)
      s.addScaledVector(F, Math.sin(t * 0.29 + 1.3) * 0.005)
    }
    return s
  }

  private updateSteps(h: number, outside: number, xiE: Vector2) {
    this.refreshChase(h)

    if (this.swing >= 0) {
      const sw = this.feet[this.swing]
      sw.t = Math.min(1, sw.t + h / sw.dur)
      if (this.stepKind === 'kick') {
        this.kickEval(sw.t, sw)
        if (!this.kickStruck && sw.t >= KICK_STRIKE) this.strike()
        if (sw.t >= 1) this.touchdown()
        return
      }
      if ((this.stepKind === 'capture' || this.stepKind === 'walk') && sw.t < 0.72) {
        const tgt = this.planTarget(this.swing, sw.dur * (1 - sw.t), this.stepKind)
        sw.to.lerp(tgt, 1 - Math.exp(-h * 16))
      } else if (this.stepKind === 'plan' && sw.t < 0.75) {
        // Where the center of pressure can't follow the plan, the capture
        // point drifts off it; move the foot by the same amount.
        const grow = Math.exp(W * sw.dur * (1 - sw.t)) - 1
        const adj = _a.copy(this.pWant).sub(this.p).multiplyScalar(grow)
        if (adj.length() > 0.1) adj.setLength(0.1)
        adj.add(this.stepTarget)
        const st = this.feet[1 - this.swing]
        const yawRef = lerpAngle(st.yaw, sw.toYaw, 0.5)
        this.constrainTo(adj, st.pos, sw.side, fwd(yawRef, _cf), left(yawRef, _cl), 0.6)
        sw.to.lerp(adj, 1 - Math.exp(-h * 16))
      }
      const e = smoothstep(sw.t)
      sw.pos.copy(sw.from).lerp(sw.to, e)
      sw.yaw = lerpAngle(sw.fromYaw, sw.toYaw, e)
      const stride = clamp(sw.from.distanceTo(sw.to) / 0.35, 0.25, 1)
      sw.lift = sw.height * Math.pow(Math.sin(Math.PI * sw.t), 0.85) + sw.lift0 * (1 - smoothstep(sw.t * 1.6))
      sw.pitch = 0.3 * stride * Math.sin(2 * Math.PI * sw.t)
      if (sw.t >= 1) this.touchdown()
      return
    }

    this.dsTime += h
    const settled = outside === 0 && this.v.length() < (this.chase ? 0.12 : 0.07) && !this.drag
    this.settledTime = settled ? this.settledTime + h : 0

    if (this.mode === 'plan') {
      if (outside < 0.1) {
        this.runPlan(h, xiE)
        return
      }
      // Knocked off the plan: catch the balance first, plan again after.
      this.leavePlan()
    }

    if (this.mode === 'walk') {
      const mid = this.mid(_a)
      const toGoal = _b.copy(this.home).sub(mid)
      const dist = toGoal.length()
      // Hand over to the balance controller near the goal; its last capture
      // step absorbs the remaining walking speed.
      const off = Math.abs(wrapAngle(Math.atan2(toGoal.x, toGoal.y) - this.heading))
      const astray = this.walkFace && off > 1.2 && off < Math.PI - 0.6
      // Watchdog: hand back to balance if the walk stops making progress.
      if (dist < this.walkBest - 0.05) {
        this.walkBest = dist
        this.walkStall = 0
      } else this.walkStall += h
      if (dist < 0.3 || astray || this.walkSteps > 30 || this.v.length() > 1.1 || this.walkStall > 2.5) {
        this.mode = 'stand'
      } else {
        toGoal.divideScalar(dist)
        this.walkDir.lerp(toGoal, 1 - Math.exp(-h * 4)).normalize()
        if (this.walkFace) this.heading = Math.atan2(this.walkDir.x, this.walkDir.y)
        if (this.walkLaunched) {
          if (this.dsTime > 0.03) this.startStep(1 - this.lastSwing, 'walk', 0)
        } else {
          this.launchTime += h
          const st = this.feet[1 - this.launchSwing]
          const center = this.footCenter(st, _a).addScaledVector(this.walkDir, 0.05)
          if (xiE.distanceTo(center) < 0.025 || this.launchTime > 0.9) {
            this.walkLaunched = true
            this.startStep(this.launchSwing, 'walk', 0)
          }
        }
        return
      }
    }

    if (outside > 0.004 && this.dsTime > 0.015) {
      this.tidyPhase = 'none'
      _b.copy(xiE).sub(this.mid(_a))
      const natural = this.chooseSwing(_b)
      // Step with whichever foot best captures the momentum. When the natural
      // foot is already at full stride, the other one shuffles in behind it.
      const T = this.captureDur(outside)
      const r0 = this.captureResidual(0, T) - (natural === 0 ? 0.03 : 0)
      const r1 = this.captureResidual(1, T) - (natural === 1 ? 0.03 : 0)
      this.startStep(r0 <= r1 ? 0 : 1, 'capture', outside)
      return
    }

    if (this.mode !== 'stand') return
    if (this.tidyPhase === 'shift') {
      // Weight goes over the stance foot before the other one lifts.
      this.tidyTime += h
      const st = this.feet[1 - this.tidySwing]
      const center = this.footCenter(st, _a)
      if ((xiE.distanceTo(center) < 0.03 && this.c.distanceTo(center) < 0.05) || this.tidyTime > 0.9) {
        this.tidyPhase = 'none'
        this.startStep(this.tidySwing, 'tidy', 0)
      }
    } else if (this.settledTime > (this.chase ? 0.15 : 0.45) && !this.drag) {
      if (!this.chase && this.ballReady()) this.startChase()
      if (this.chase) {
        if (this.ballResting() && !this.makePlan(-1)) this.endChase(3)
        return
      }
      const mid = this.mid(_a)
      const d = _b.copy(this.home).sub(mid)
      const dist = d.length()
      d.divideScalar(dist || 1)
      // Off the spot or turned away: plan the steps back in one go.
      const turned = Math.max(
        Math.abs(wrapAngle(this.feet[0].yaw - this.homeYaw)),
        Math.abs(wrapAngle(this.feet[1].yaw - this.homeYaw)),
      )
      if ((dist > HOME_R || turned > 0.4) && this.planHome(-1)) return
      // Too far for a plan: the walking controller. Sideways walking is
      // unstable for a biped, so far from home it turns in place to face it
      // and walks forward; near home it turns to the home heading.
      const far = dist > (this.tripFace === null ? HOME_R : 0.25)
      if (!far) this.tripFace = null
      else if (this.tripFace === null) {
        this.tripFace = dist > 0.9 || Math.abs(d.dot(fwd(this.homeYaw, _f))) < 0.75
      }
      const face = far && this.tripFace === true
      this.heading = face ? Math.atan2(d.x, d.y) : this.homeYaw
      const yawErr = Math.max(
        Math.abs(wrapAngle(this.feet[0].yaw - this.heading)),
        Math.abs(wrapAngle(this.feet[1].yaw - this.heading)),
      )
      if (far && yawErr < 0.3 && this.settledTime > 0.7) {
        this.mode = 'walk'
        this.walkLaunched = false
        this.walkSteps = 0
        this.walkFace = face
        this.walkDir.copy(d)
        this.launchSwing = this.chooseSwing(d)
        this.launchTime = 0
        this.walkBest = dist
        this.walkStall = 0
      } else {
        const pick = this.tidyPick()
        if (pick >= 0) this.shiftFor(pick)
      }
    }
  }

  private shiftFor(i: number) {
    this.tidyPhase = 'shift'
    this.tidySwing = i
    this.tidyTime = 0
  }

  // ------------------------------------------------------------------ ball

  /** Grounded and nearly still, so it can be walked up to. */
  private ballSlow(): boolean {
    const b = this.ball
    if (!b || !b.active || b.held) return false
    return b.pos.y < BALL_R + 0.05 && Math.hypot(b.vel.x, b.vel.z) < 0.35 && Math.abs(b.vel.y) < 0.5
  }

  /** Done bouncing, so a plan made now will hold. */
  private ballResting(): boolean {
    const b = this.ball!
    return this.ballSlow() && b.pos.y < BALL_R + 0.01 && Math.abs(b.vel.y) < 0.1
  }

  /** Where a rolling ball comes to rest (rolling resistance as in ball.ts). */
  private ballRest(out: Vector2): Vector2 {
    const b = this.ball!
    const sp = Math.hypot(b.vel.x, b.vel.z)
    out.set(b.pos.x, b.pos.z)
    if (sp < 1e-3) return out
    const d = sp / 0.25 - (0.35 / 0.0625) * Math.log(1 + (0.25 * sp) / 0.35)
    return out.add(_g.set(b.vel.x, b.vel.z).multiplyScalar(d / sp))
  }

  private ballReady(): boolean {
    return this.chaseCooldown <= 0 && !!this.ball?.reachable && this.ballResting()
  }

  private startChase() {
    this.chase = true
    this.planCount = 0
    this.gest.cancel()
    this.tripFace = null
  }

  private refreshChase(h: number) {
    this.chaseCooldown -= h
    if (!this.chase) return
    const b = this.ball
    if (!b || !b.active || !b.reachable || b.held || this.air) {
      this.endChase(0.6)
      return
    }
    if (this.mode !== 'plan' || this.planGoal !== 'ball' || (this.swing >= 0 && this.stepKind === 'kick')) return
    // It moved: plan again from the next footfall.
    if (!this.ballSlow() || this.ballRest(_b).distanceTo(this.kickBall) > 0.1) this.replan = true
  }

  private endChase(cooldown: number) {
    this.chase = false
    this.chaseCooldown = cooldown
    this.tripFace = null
    if (this.mode === 'stand') this.heading = this.homeYaw
  }

  /**
   * Footsteps to the ball: a curve that ends lined up behind it, the plant
   * beside it, then the kick. `first` forces which foot leads (-1: either).
   */
  private makePlan(first: number): boolean {
    const b = this.ball
    if (!b || ++this.planCount > 6) return false
    const B = this.ballRest(new Vector2())
    const M0 = this.mid(new Vector2())
    const yaw0 = lerpAngle(this.feet[0].yaw, this.feet[1].yaw, 0.5)

    // Kick along the way in, bent toward the viewer, so the run-up is straight.
    const toBall = B.clone().sub(M0)
    if (toBall.lengthSq() < 1e-6) fwd(yaw0, toBall)
    toBall.normalize()
    const toCam = new Vector2(this.viewer.x - B.x, this.viewer.z - B.y).normalize()
    const dir = toBall.addScaledVector(toCam, 0.4)
    // Never away from the viewer, where the ball would vanish: across instead.
    const away = dir.dot(toCam)
    if (away < 0) dir.addScaledVector(toCam, -away)
    if (dir.lengthSq() < 0.04) dir.set(toCam.y, -toCam.x).multiplyScalar(Math.random() < 0.5 ? 1 : -1)
    const yawK = Math.atan2(dir.x, dir.y) + (Math.random() * 2 - 1) * 0.2
    const F = fwd(yawK, this.kickDir)
    const L = left(yawK, new Vector2())
    // Plant on the near side of the ball; the far foot kicks.
    const kick = M0.clone().sub(B).dot(L) > 0 ? 1 : 0
    const sP = this.feet[1 - kick].side
    const plant = B.clone().addScaledVector(F, -PLANT_BACK).addScaledVector(L, sP * STANCE_W)
    const Mf = plant.addScaledVector(L, (-sP * STANCE_W) / 2)
    const Mk = Mf.clone().addScaledVector(F, PLANT_BACK - KICK_GAP)

    const frames = this.approachFrames(M0, yaw0, Mk, yawK)
    frames.push({ p: Mf, yaw: yawK })
    // Feet alternate and the plant foot steps last; one extra step lets the
    // preferred foot lead.
    let lead = frames.length % 2 === 1 ? 1 - kick : kick
    const pref = first >= 0 ? first : M0.distanceTo(Mk) > 0.15 ? this.chooseSwing(Mk.clone().sub(M0)) : -1
    if (pref >= 0 && pref !== lead) {
      frames.unshift({ p: M0.clone().lerp(frames[0].p, 0.5), yaw: lerpAngle(yaw0, frames[0].yaw, 0.5) })
      lead = pref
    }
    if (frames.length > 28) return false
    this.followFrames(frames, lead)
    // The kick carries through into a step past the plant foot.
    const plantPos = this.plan[this.plan.length - 1].pos
    const land = plantPos.clone().addScaledVector(L, -sP * STANCE_W).addScaledVector(F, 0.26)
    this.plan.push({ foot: kick, pos: land, yaw: yawK, kind: 'kick', dur: KICK_T })
    this.kickBall.copy(B)
    this.startPlan('ball')
    return true
  }

  /** Walk back to the home spot and face the viewer, feet together. */
  private planHome(first: number): boolean {
    const M0 = this.mid(new Vector2())
    const yaw0 = lerpAngle(this.feet[0].yaw, this.feet[1].yaw, 0.5)
    const frames = this.approachFrames(M0, yaw0, this.home, this.homeYaw)
    frames.push({ p: this.home.clone(), yaw: this.homeYaw })
    if (frames.length > 30) return false
    const d = this.home.clone().sub(M0)
    const lead = first >= 0 ? first : d.length() > 0.15 ? this.chooseSwing(d) : 1 - this.lastSwing
    this.followFrames(frames, lead)
    this.startPlan('home')
    return true
  }

  /** Footsteps for mid-stance frames, alternating from `lead`, each clear of the stance foot. */
  private followFrames(frames: Frame[], lead: number) {
    const plan: PlanStep[] = []
    const prev = [this.feet[0].pos.clone(), this.feet[1].pos.clone()]
    const prevYaw = [this.feet[0].yaw, this.feet[1].yaw]
    let foot = lead
    for (const fr of frames) {
      const st = 1 - foot
      const side = this.feet[foot].side
      const yaw = prevYaw[st] + clamp(wrapAngle(fr.yaw - prevYaw[st]), -0.6, 0.6)
      const yawRef = lerpAngle(prevYaw[st], yaw, 0.5)
      const pos = fr.p.clone().addScaledVector(left(fr.yaw, _g), (side * STANCE_W) / 2)
      this.constrainTo(pos, prev[st], side, fwd(yawRef, _cf), left(yawRef, _cl), 0.55)
      const dur = pos.distanceTo(prev[foot]) < 0.12 ? 0.34 : STEP_T
      plan.push({ foot, pos, yaw, kind: 'plan', dur })
      prev[foot] = pos
      prevYaw[foot] = yaw
      foot = st
    }
    this.plan = plan
  }

  private startPlan(goal: 'ball' | 'home') {
    this.planGoal = goal
    this.mode = 'plan'
    this.planShift = true
    this.planWait = 0
    this.replan = false
    this.tidyPhase = 'none'
    this.planRefs()
  }

  /** Mid-stance frames from here to (M1, yaw1), excluding the start. */
  private approachFrames(M0: Vector2, yaw0: number, M1: Vector2, yaw1: number): Frame[] {
    const out: Frame[] = []
    const chord = M0.distanceTo(M1)
    const D0 = M1.clone().sub(M0).divideScalar(chord || 1)
    // Stepping straight there while turning: short strides sideways or back.
    const ym = lerpAngle(yaw0, yaw1, 0.5)
    const fw = D0.dot(fwd(ym, _f)) * chord
    const lat = D0.dot(left(ym, _l)) * chord
    const nDirect = Math.max(
      1,
      Math.ceil(fw > 0 ? fw / 0.22 : -fw / 0.14),
      Math.ceil(Math.abs(lat) / 0.12),
      Math.ceil(Math.abs(wrapAngle(yaw1 - yaw0)) / 0.45),
    )
    // Turning to face the way first, then walking a curve that arrives lined up.
    const turn = wrapAngle(Math.atan2(D0.x, D0.y) - yaw0)
    const nTurn = Math.abs(turn) > 0.7 ? Math.ceil((Math.abs(turn) - 0.3) / 0.5) : 0
    const nFace = nTurn + Math.max(2, Math.ceil((chord * 1.15) / 0.24))
    if (chord < 0.6 || (chord < 1.6 && nDirect <= nFace + 1)) {
      for (let k = 1; k <= nDirect; k++) {
        out.push({ p: M0.clone().lerp(M1, k / nDirect), yaw: lerpAngle(yaw0, yaw1, k / nDirect) })
      }
      return out
    }
    const P1 = M0.clone().addScaledVector(D0, chord * 0.35)
    const P2 = M1.clone().addScaledVector(fwd(yaw1), -chord * 0.35)
    const N = 40
    const pts: Vector2[] = []
    const acc = [0]
    for (let i = 0; i <= N; i++) {
      pts.push(bezier(M0, P1, P2, M1, i / N, new Vector2()))
      if (i) acc.push(acc[i - 1] + pts[i].distanceTo(pts[i - 1]))
    }
    const len = acc[N]
    // Big turns happen on the spot first.
    let yaw = yaw0
    for (let k = 1; k <= nTurn; k++) {
      yaw = yaw0 + ((turn - Math.sign(turn) * 0.3) * k) / nTurn
      out.push({ p: M0.clone(), yaw })
    }
    const n = Math.max(2, Math.ceil(len / 0.24))
    let i = 1
    for (let k = 1; k <= n; k++) {
      const s = (len * k) / n
      while (i < N && acc[i] < s) i++
      const u = (s - acc[i - 1]) / (acc[i] - acc[i - 1] || 1)
      const p = pts[i - 1].clone().lerp(pts[i], u)
      const tan = _b.copy(pts[i]).sub(pts[i - 1])
      let ty = k === n ? yaw1 : Math.atan2(tan.x, tan.y)
      // Ease from the current facing into the path.
      if (k <= 2) ty = lerpAngle(yaw, ty, k / 2)
      out.push({ p, yaw: ty })
    }
    return out
  }

  /** Capture point reference: backward from the end of the plan to its start. */
  private planRefs() {
    const P = this.plan
    const n = P.length
    const cops: Vector2[] = [this.footCenter(this.feet[1 - P[0].foot], new Vector2())]
    for (let j = 1; j < n; j++) cops.push(this.footCenter(P[j - 1], new Vector2()))
    // It comes to rest between the last two feet, or after a kick mostly
    // still over the plant foot.
    const xi = cops[n - 1].clone().lerp(this.footCenter(P[n - 1], _a), P[n - 1].kind === 'kick' ? 0.35 : 0.5)
    for (let j = n - 1; j >= 0; j--) xi.sub(cops[j]).multiplyScalar(Math.exp(-W * P[j].dur)).add(cops[j])
    this.planCop.copy(cops[0])
    this.planXi0.copy(xi)
  }

  /** In double support while following the plan: start its next step. */
  private runPlan(h: number, xi: Vector2) {
    const ball = this.planGoal === 'ball'
    if (!this.plan.length || (ball && !this.chase)) {
      this.leavePlan()
      return
    }
    if (!ball && this.ballReady()) {
      // A ball turned up on the way home: go for it from this footfall.
      this.startChase()
      if (!this.makePlan(1 - this.lastSwing)) {
        this.endChase(3)
        this.leavePlan()
      }
      return
    }
    if (this.replan) {
      // Rolling away: stop and wait. Still settling: carry on until it rests.
      if (!this.ballSlow()) {
        this.leavePlan()
        return
      }
      if (this.ballResting()) {
        this.replan = false
        if (!this.makePlan(1 - this.lastSwing)) {
          this.leavePlan()
          return
        }
      }
    }
    if (this.planShift) {
      this.planWait += h
      if (xi.distanceTo(this.planXi0) > 0.025 && this.planWait < 0.7) return
      this.planShift = false
    } else if (this.dsTime < 0.03) return
    const s = this.plan[0]
    if (s.kind === 'kick') {
      // Swing only if the ball is still where the plant foot expects it.
      const b = this.ball!
      const st = this.feet[1 - s.foot]
      const L = left(Math.atan2(this.kickDir.x, this.kickDir.y), _l)
      _a.copy(st.pos).addScaledVector(this.kickDir, PLANT_BACK).addScaledVector(L, -st.side * STANCE_W)
      if (Math.hypot(b.pos.x - _a.x, b.pos.z - _a.y) > 0.12) {
        this.replan = true
        return
      }
      this.ballRest(this.kickBall)
    }
    this.startStep(s.foot, s.kind, 0)
  }

  private leavePlan() {
    this.mode = 'stand'
    this.plan.length = 0
    this.settledTime = 0
    if (!this.chase) this.heading = this.homeYaw
  }

  /** Wind up, swing through the ball, follow through, land ahead. */
  private planKick(sw: Foot, s: PlanStep) {
    const F = this.kickDir
    const B = this.kickBall
    const a0 = sw.pos.clone()
    const back = a0.clone().addScaledVector(F, -0.12)
    const hit = B.clone().addScaledVector(F, -(BALL_R + DIM.toe - 0.04))
    // The leg carries on up and out in front, nearly straight.
    const through = B.clone().addScaledVector(F, 0.6)
    this.kickKeys = [
      { t: 0, x: a0.x, z: a0.y, y: 0, pitch: 0 },
      { t: 0.28, x: back.x, z: back.y, y: 0.22, pitch: 1.0 },
      { t: KICK_STRIKE, x: hit.x, z: hit.y, y: 0.035, pitch: 0.25 },
      { t: 0.74, x: through.x, z: through.y, y: 0.3, pitch: -0.1 },
      { t: 1, x: s.pos.x, z: s.pos.y, y: 0, pitch: 0 },
    ]
    sw.to.copy(s.pos)
    sw.toYaw = s.yaw
    this.kickStruck = false
  }

  /** Cubic Hermite through the kick keys with Catmull-Rom tangents. */
  private kickEval(t: number, sw: Foot) {
    const K = this.kickKeys
    let i = 0
    while (i < K.length - 2 && t > K[i + 1].t) i++
    const k0 = K[i]
    const k1 = K[i + 1]
    const dt = k1.t - k0.t
    const u = clamp((t - k0.t) / dt, 0, 1)
    const u2 = u * u
    const u3 = u2 * u
    const h00 = 2 * u3 - 3 * u2 + 1
    const h10 = u3 - 2 * u2 + u
    const h01 = -2 * u3 + 3 * u2
    const h11 = u3 - u2
    const tan = (j: number, k: keyof Key) =>
      j === 0 || j === K.length - 1 ? 0 : (K[j + 1][k] - K[j - 1][k]) / (K[j + 1].t - K[j - 1].t)
    const val = (k: keyof Key) => h00 * k0[k] + h10 * dt * tan(i, k) + h01 * k1[k] + h11 * dt * tan(i + 1, k)
    sw.pos.set(val('x'), val('z'))
    sw.lift = Math.max(0, val('y'))
    sw.pitch = val('pitch')
    sw.yaw = lerpAngle(sw.fromYaw, sw.toYaw, smoothstep(t))
  }

  private strike() {
    this.kickStruck = true
    const b = this.ball
    if (b && b.active && !b.held && b.pos.y < 0.35) {
      if (Math.hypot(b.pos.x - this.kickBall.x, b.pos.z - this.kickBall.y) < 0.22) {
        const vh = 4.4 + Math.random() * 1.6
        const vv = 1.6 + Math.random() * 1.6
        b.kick(new Vector3(this.kickDir.x * vh, vv, this.kickDir.y * vh))
      }
    }
    this.endChase(1.5)
  }

  /** The kicking leg passes through the ball until the strike. */
  get ghostLeg(): number {
    if (this.swing >= 0 && this.stepKind === 'kick' && this.feet[this.swing].t < 0.85) return this.swing
    return -1
  }

  // -------------------------------------------------------------- stepping

  private chooseSwing(d: Vector2): number {
    const len = d.length()
    if (len < 1e-4) return 1 - this.lastSwing
    const L = left(this.pelvisYaw, _l)
    const dl = d.dot(L)
    if (Math.abs(dl) > 0.45 * len) return dl > 0 ? 0 : 1
    const p0 = this.feet[0].pos.dot(d) / len
    const p1 = this.feet[1].pos.dot(d) / len
    if (Math.abs(p0 - p1) < 0.03) return 1 - this.lastSwing
    return p0 < p1 ? 0 : 1
  }

  private planYaw(i: number): number {
    const st = this.feet[1 - i]
    const max = this.mode === 'stand' ? 0.6 : 0.5
    return st.yaw + clamp(wrapAngle(this.heading - st.yaw), -max, max)
  }

  private planTarget(i: number, tRem: number, kind: StepKind): Vector2 {
    const sw = this.feet[i]
    const st = this.feet[1 - i]
    const yawT = this.planYaw(i)
    const yawRef = lerpAngle(st.yaw, yawT, 0.5)
    const F = fwd(yawRef, new Vector2())
    const L = left(yawRef, new Vector2())
    const target = new Vector2()

    if (kind === 'tidy') {
      return target.copy(st.pos).addScaledVector(L, sw.side * STANCE_W)
    }

    const xi = _v3a.set(this.v.x / W + this.c.x, 0, this.v.y / W + this.c.y)
    const xiE = new Vector2(xi.x + this.ext.x / W2, xi.z + this.ext.y / W2)
    const ps = new Vector2()
    if (kind === 'walk') this.footCenter(st, ps)
    else clampToPoly(xiE, this.footPoly(st), ps)
    target.copy(xiE).sub(ps).multiplyScalar(Math.exp(W * tRem)).add(ps)
    target.addScaledVector(fwd(yawT, _f), -FOOT_CENTER)
    if (kind === 'walk') {
      const mid = this.mid(_a)
      // Shorter strides when walking sideways or backward.
      const along = this.walkDir.dot(fwd(yawRef, _f))
      const maxL = along > 0.7 ? 0.24 : along > -0.3 ? 0.15 : 0.16
      // Strides shrink to zero near the goal, so the last steps brake to a stop.
      const dist = this.home.distanceTo(mid)
      const L0 = dist < 0.2 ? 0 : clamp(dist * 0.5, 0.08, maxL)
      target.addScaledVector(this.walkDir, -L0 / (Math.exp(W * WALK_T) - 1))
    }
    target.addScaledVector(L, sw.side * (kind === 'walk' ? 0.035 : W_OUT))
    // Pulled along, it shuffles rather than doing the splits.
    return this.constrainTo(target, st.pos, sw.side, F, L, this.drag ? 0.5 : MAX_STEP)
  }

  /** Keep a foot target outside the stance foot and within one stride of it. */
  private constrainTo(target: Vector2, st: Vector2, side: number, F: Vector2, L: Vector2, max: number): Vector2 {
    const rx = target.x - st.x
    const rz = target.y - st.y
    let lat = (rx * L.x + rz * L.y) * side
    let fw = rx * F.x + rz * F.y
    lat = Math.max(lat, MIN_LAT)
    const len = Math.hypot(lat, fw)
    if (len > max) {
      lat = Math.max(lat * (max / len), MIN_LAT)
      fw = Math.sign(fw) * Math.sqrt(Math.max(0, max * max - lat * lat))
    }
    return target
      .copy(st)
      .addScaledVector(L, side * lat)
      .addScaledVector(F, fw)
  }

  private captureDur(urgency: number): number {
    return clamp(0.36 - urgency * 0.4, 0.22, 0.36)
  }

  /** How far the capture point would still be from the support after stepping foot i. */
  private captureResidual(i: number, T: number): number {
    const st = this.feet[1 - i]
    const target = this.planTarget(i, T, 'capture')
    const xiE = new Vector2(
      this.c.x + this.v.x / W + this.ext.x / W2,
      this.c.y + this.v.y / W + this.ext.y / W2,
    )
    const ps = new Vector2()
    clampToPoly(xiE, this.footPoly(st), ps)
    const xiT = xiE.sub(ps).multiplyScalar(Math.exp(W * T)).add(ps)
    const pts: Vector2[] = []
    this.footCorners(st, pts)
    this.footCorners({ pos: target, yaw: this.planYaw(i) }, pts)
    return clampToPoly(xiT, convexHull(pts), new Vector2())
  }

  private startStep(i: number, kind: StepKind, urgency: number) {
    const sw = this.feet[i]
    sw.planted = false
    sw.from.copy(sw.pos)
    sw.fromYaw = sw.yaw
    sw.t = 0
    sw.lift0 = 0
    if (kind === 'plan' || kind === 'kick') {
      const s = this.plan[0]
      sw.dur = s.dur
      if (kind === 'kick') this.planKick(sw, s)
      else {
        sw.to.copy(s.pos)
        sw.toYaw = s.yaw
        this.stepTarget.copy(s.pos)
      }
    } else {
      sw.dur = kind === 'capture' ? this.captureDur(urgency) : kind === 'walk' ? WALK_T : 0.42
      sw.toYaw = this.planYaw(i)
      sw.to.copy(this.planTarget(i, sw.dur, kind))
    }
    const dist = sw.to.distanceTo(sw.from)
    sw.height =
      kind === 'tidy' ? 0.035 + dist * 0.06 : kind === 'plan' ? 0.04 + dist * 0.07 : clamp(0.045 + dist * 0.12, 0.05, 0.11)
    this.swing = i
    this.lastSwing = i
    this.stepKind = kind
    if (kind === 'walk') this.walkSteps++
  }

  private touchdown() {
    const sw = this.feet[this.swing]
    sw.planted = true
    sw.pos.copy(sw.to)
    sw.yaw = sw.toYaw
    sw.lift = 0
    sw.pitch = 0
    this.swing = -1
    this.dsTime = 0
    if (this.mode === 'plan') {
      this.plan.shift()
      if (this.plan.length) this.planRefs()
    }
  }

  /** Which foot to move to restore a neutral stance, or -1 if none. */
  private tidyPick(): number {
    const yaw = this.heading
    const L = left(yaw, _l)
    const [f0, f1] = this.feet
    const desired = _b.copy(f1.pos).addScaledVector(L, STANCE_W)
    const posErr = f0.pos.distanceTo(desired)
    const y0 = Math.abs(wrapAngle(f0.yaw - yaw))
    const y1 = Math.abs(wrapAngle(f1.yaw - yaw))
    if (posErr < 0.045 && y0 < 0.12 && y1 < 0.12) return -1
    if (Math.abs(y0 - y1) > 0.1) return y0 > y1 ? 0 : 1
    const d0 = this.footCenter(f0, _a).distanceTo(this.c)
    const d1 = this.footCenter(f1, _a).distanceTo(this.c)
    return d0 > d1 ? 0 : 1
  }

  private updateSprings(h: number) {
    const yaw = this.pelvisYaw
    const F = fwd(yaw, _f)
    const L = left(yaw, _l)
    const ground = !this.air

    // Pelvis yaw follows the feet.
    if (ground) {
      let yawT: number
      if (this.swing < 0) yawT = lerpAngle(this.feet[0].yaw, this.feet[1].yaw, 0.5)
      else {
        const sw = this.feet[this.swing]
        const st = this.feet[1 - this.swing]
        yawT = lerpAngle(st.yaw, sw.toYaw, 0.5 * smoothstep(sw.t))
      }
      this.pelvisYaw = lerpAngle(this.pelvisYaw, yawT, 1 - Math.exp(-h * 8))
    }

    // Upper body lags behind accelerations of the base. A pull tips it about
    // the pelvis: toward the pull when held high, away when held low, and
    // down to one side when pushed down off-center.
    _lean.copy(this.aFilt).sub(this.ext).multiplyScalar(-0.02)
    let twistT = 0
    let leanMax = 0.35
    if (this.drag && ground) {
      const r = this.lever
      const P = this.pull
      const py = clamp(P.y, -12, 12)
      _lean.x += (this.ext.x * r.y - r.x * py) * 0.09
      _lean.y += (this.ext.y * r.y - r.z * py) * 0.09
      twistT = clamp((r.z * this.ext.x - r.x * this.ext.y) * 0.1, -0.45, 0.45)
      leanMax = 0.45
    }
    if (this.mode === 'walk' && this.walkLaunched) _lean.addScaledVector(this.walkDir, 0.05)
    let kickArc = 0
    if (this.swing >= 0 && this.stepKind === 'kick') {
      const sw = this.feet[this.swing]
      const t = sw.t
      kickArc = Math.sin(Math.PI * t)
      // Lean back through the strike.
      const k = (t - KICK_STRIKE) / 0.15
      _lean.addScaledVector(this.kickDir, -0.08 * Math.exp(-k * k))
      // Shoulders counter the hips: wound up on the backswing, unwinding through the ball.
      const s =
        t < KICK_STRIKE
          ? -0.6 * Math.sin((Math.PI * t) / KICK_STRIKE)
          : Math.sin((Math.PI * (t - KICK_STRIKE)) / (1 - KICK_STRIKE))
      twistT += sw.side * 0.22 * s
    }
    this.kickArc = kickArc
    this.leanP.step(clamp(_lean.dot(F), -leanMax, leanMax), 70, 11, h)
    this.leanR.step(clamp(-_lean.dot(L), -leanMax + 0.05, leanMax - 0.05), 70, 11, h)
    this.twist.step(twistT, 45, 8, h)

    // Arms: counter-swing to the opposite leg, hang back from the body's
    // acceleration like pendulums, flare a little when knocked, go out for
    // balance during a kick or a fall. Gestures blend over the top.
    const G = this.gest
    G.update(h)
    const hp = _hp.copy(this.v).sub(this.vSlow)
    const hpMag = Math.min(hp.length(), 1.5)
    const accF = clamp(this.aFilt.dot(F) * 0.07, -0.6, 0.6)
    const accL = clamp(this.aFilt.dot(L) * 0.07, -0.6, 0.6)
    const flare = this.drag || this.air ? 0.15 : 0.5
    const free = this.air ? this.air.free : 0
    for (const i of [0, 1]) {
      const side = this.feet[i].side
      const opp = this.feet[1 - i]
      const oppFwd = this.air ? 0 : (opp.pos.x - this.c.x) * F.x + (opp.pos.y - this.c.y) * F.y + FOOT_CENTER
      let p = 0.04 - 1.3 * oppFwd + accF + flare * hp.dot(F)
      let a = 0.1 + flare * hpMag - side * accL + 0.45 * free
      let el = -0.22 - flare * 0.7 * hpMag - 0.4 * free
      if (kickArc > 0) {
        // The kicking side's arm goes back, the other one out wide.
        if (i === this.swing) {
          p += 0.3 * kickArc
          a += 0.15 * kickArc
        } else {
          a += 0.55 * kickArc
          el -= 0.3 * kickArc
        }
      }
      let tw = 0
      let wp = 0
      const grabbed = this.drag && this.drag.arm === i && this.armAim
      if (grabbed) [p, a, el] = this.armAim!
      const w = grabbed ? 0 : G.armW[i]
      if (w > 1e-3) {
        const P = G.arm[i]
        p += (P.p - p) * w
        a += (P.a - a) * w
        el += (P.el - el) * w
        tw = P.tw * w
        wp = P.wp * w
      }
      // A held arm follows the hand closely.
      const k = grabbed ? 320 : 90 + 160 * w
      const d = grabbed ? 32 : 10 + 10 * w
      // A held arm can be pulled all the way overhead.
      this.armP[i].step(clamp(p, grabbed ? -3 : -2.6, grabbed ? 1.2 : 0.9), k, d, h)
      this.armA[i].step(clamp(a, grabbed ? -0.3 : 0.04 - 0.5 * w, grabbed ? 3.2 : 1.9), k, d, h)
      this.armE[i].step(clamp(el, -2.3, -0.03), k + 20, d + 2, h)
      this.armT[i].step(tw, k, d, h)
      this.armWP[i].step(wp, k, d, h)
      const cu = this.curl[i]
      for (let f = 0; f < 4; f++) cu[f] = damp(cu[f], G.arm[i].curl[f] * w, 16, h)
    }

    // Head follows the look target, relative to the body.
    let [hy, hpch] = this.headAngles(this.lookTarget, F, L)
    if (G.lookW > 1e-3) {
      let gy = 0
      let gp = 0
      if (G.look === 'viewer') [gy, gp] = this.headAngles(this.viewer, F, L)
      else if (G.look === 'hand') [gy, gp] = this.headAngles(this.palmW[G.lookArm], F, L)
      gy += G.lookYaw
      gp += G.lookPitch
      hy += (gy - hy) * G.lookW
      hpch += (gp - hpch) * G.lookW
    }
    this.headYaw.step(clamp(hy, -1.1, 1.1) - this.twist.x, 55, 12, h)
    this.headPitch.step(clamp(hpch, -0.45, 0.8), 55, 12, h)
    this.headRoll.step(G.roll, 50, 11, h)
    this.shrug.step(G.shrug, 160, 16, h)
  }

  /** Head yaw and pitch, relative to the pelvis, to look at a point. */
  private headAngles(t: Vector3, F: Vector2, L: Vector2): [number, number] {
    const rx = t.x - this.c.x
    const ry = t.y - (this.pelvisH + 0.67)
    const rz = t.z - this.c.y
    const lx = rx * L.x + rz * L.y
    const lz = rx * F.x + rz * F.y
    let hy = Math.atan2(lx, lz)
    if (Math.abs(hy) > 1.9) hy = 0
    return [hy, Math.atan2(-ry, Math.hypot(lx, lz))]
  }

  // ----------------------------------------------------------------- pose

  apply(rig: Rig, dt: number) {
    const A = this.air
    const yaw = this.pelvisYaw
    const L = left(yaw, _l)
    const root = rig.root
    // On its toes when pulled up.
    const tip = A ? 0 : Math.max(this.squat.x, 0)

    if (A) {
      root.position.copy(A.pos)
      root.quaternion.copy(A.q)
    } else {
      // Pelvis height: as tall as the planted legs allow, lower when
      // stepping urgently, walking, kicking or pushed down.
      let maxH = Infinity
      for (const f of this.feet) {
        const hx = this.c.x + L.x * f.side * DIM.hipX
        const hz = this.c.y + L.y * f.side * DIM.hipX
        const dh = Math.hypot(f.pos.x - hx, f.pos.y - hz)
        const reach = dh < LEG_MAX ? Math.sqrt(LEG_MAX * LEG_MAX - dh * dh) : 0
        const lim = DIM.ankleH + f.lift + (f.planted ? tip : 0) + reach - DIM.hipY
        if (f.planted || f.t > 0.75) maxH = Math.min(maxH, lim)
      }
      const hpMag = Math.min(_hp.copy(this.v).sub(this.vSlow).length(), 1)
      const breath = this.reduced ? 0 : Math.sin(this.time * 1.5) * 0.0015
      const crouch = (this.mode === 'walk' && this.walkLaunched ? 0.015 : 0) + 0.035 * hpMag + 0.035 * this.kickArc
      const want = Math.min(DIM.standPelvis - crouch + breath + this.squat.x, maxH)
      this.pelvisH = clamp(damp(this.pelvisH, want, 30, dt), 0.68, Math.max(maxH, 0.68))

      let roll = this.leanR.x * 0.25
      if (this.swing >= 0) roll -= this.feet[this.swing].side * 0.035 * Math.sin(Math.PI * this.feet[this.swing].t)
      root.position.set(this.c.x, this.pelvisH, this.c.y)
      _e.set(this.leanP.x * 0.25, yaw, roll, 'YXZ')
      root.quaternion.setFromEuler(_e)
    }

    rig.spine.rotation.set(this.leanP.x * 0.75 + (this.reduced ? 0 : Math.sin(this.time * 1.5) * 0.006), this.twist.x, this.leanR.x * 0.75, 'YXZ')
    rig.neck.rotation.set(this.headPitch.x * 0.35, this.headYaw.x * 0.4, this.headRoll.x * 0.4)
    rig.head.rotation.set(this.headPitch.x * 0.65, this.headYaw.x * 0.6, this.headRoll.x * 0.6)

    for (const i of [0, 1]) {
      const side = this.feet[i].side
      const sh = rig.shoulder[i]
      sh.position.y = DIM.shoulderY + this.shrug.x * 0.03
      sh.rotation.set(this.armP[i].x, this.armT[i].x, side * this.armA[i].x)
      rig.elbow[i].rotation.set(this.armE[i].x, 0, 0)
      rig.wrist[i].rotation.set(-0.12 + this.armWP[i].x, 0, 0)
      const cu = this.curl[i]
      const fingers = rig.fingers[i]
      for (let f = 0; f < 4; f++) {
        fingers[f][0].rotation.z = -side * (0.22 + 1.25 * cu[f])
        fingers[f][1].rotation.z = -side * (0.42 + 1.35 * cu[f])
      }
      const avg = (cu[0] + cu[1] + cu[2] + cu[3]) / 4
      rig.thumb[i][1].rotation.z = -side * (0.3 + 0.9 * Math.max(avg, -0.2))
    }

    root.updateMatrixWorld(true)
    rig.head.localToWorld(this.headW.set(0, 0.1, 0.06))

    // Roll each wrist so the palm faces where the gesture wants it.
    for (const i of [0, 1]) {
      const P = this.gest.arm[i]
      const w = this.gest.armW[i]
      const wr = rig.wrist[i]
      let want = 0
      if (w > 1e-3 && P.palm) {
        wr.getWorldPosition(_v3a)
        rig.elbow[i].getWorldQuaternion(_qTmp).multiply(_qX.setFromAxisAngle(X_AXIS, wr.rotation.x))
        const D = _v3b.copy(this.headW).sub(_v3a)
        if (P.palm === 'back') D.negate()
        D.applyQuaternion(_qTmp.invert())
        const s = this.feet[i].side
        want = Math.atan2(s * D.z, -s * D.x)
      }
      this.palmRoll[i] = lerpAngle(this.palmRoll[i], want, 1 - Math.exp(-dt * 12))
      const roll = this.palmRoll[i] * Math.min(w, 1)
      if (roll !== wr.rotation.y) {
        wr.rotation.y = roll
        wr.updateMatrixWorld(true)
      }
      wr.localToWorld(this.palmW[i].set(0, -0.065, 0))
    }

    _qRoot.copy(root.quaternion)
    _qRootInv.copy(_qRoot).invert()

    for (const i of [0, 1]) {
      const f = this.feet[i]
      rig.hip[i].getWorldPosition(_H)
      if (A) {
        _A.copy(A.feet[i].p)
        _pole.set(f.side * 0.15, 0, 1).applyQuaternion(A.q)
      } else {
        _A.set(f.pos.x, DIM.ankleH + f.lift + (f.planted ? tip : 0), f.pos.y)
        _pole.set(Math.sin(f.yaw), 0, Math.cos(f.yaw))
        _pole.x += Math.sin(yaw) + L.x * f.side * 0.15
        _pole.z += Math.cos(yaw) + L.y * f.side * 0.15
      }
      this.solveLeg(_H, _A, _pole)
      rig.hip[i].quaternion.copy(_qRootInv).multiply(_qThigh)
      rig.knee[i].quaternion.copy(_qTmp.copy(_qThigh).invert()).multiply(_qShin)
      if (A) _qFoot.copy(A.q).multiply(_qX.setFromAxisAngle(X_AXIS, f.pitch))
      else _qFoot.setFromEuler(_e.set(f.pitch + (f.planted ? tip * 6 : 0), f.yaw, 0, 'YXZ'))
      rig.ankle[i].quaternion.copy(_qTmp.copy(_qShin).invert()).multiply(_qFoot)
    }

    if (this.drag && this.drag.arm >= 0) this.reachFor(rig, this.drag.arm, this.drag.target, this.drag.reach)
    root.updateMatrixWorld(true)
  }

  /**
   * Shoulder pitch, abduction and elbow bend that put the held point of arm
   * i on the pointer (two-link IK in the spine's frame).
   */
  private reachFor(rig: Rig, i: number, target: Vector3, reach: number) {
    rig.shoulder[i].getWorldPosition(_v3a)
    const H = _v3b.copy(target).sub(_v3a)
    rig.spine.getWorldQuaternion(_qTmp)
    H.applyQuaternion(_qTmp.invert())
    const len = H.length()
    if (len < 1e-4) return
    H.divideScalar(len)
    const a = DIM.upperArm
    const b = Math.max(reach - a, 0.05)
    const d = clamp(len, Math.abs(a - b) + 0.02, a + b - 1e-3)
    // The elbow bends forward; the hand sits at angle beta from the upper arm.
    const el = -Math.acos(clamp((d * d - a * a - b * b) / (2 * a * b), -1, 1))
    const beta = Math.atan2(-b * Math.sin(el), a + b * Math.cos(el))
    // The shoulder turns as Rz(side * abduction) Rx(pitch) (Euler ZXY), taking
    // the hand from (0, -cos beta, sin beta) to H. Of the two solutions, take
    // the one in range closest to where the arm is, so it never flips.
    const side = this.feet[i].side
    const g = Math.asin(clamp(H.z, -1, 1))
    let best = Infinity
    for (const [gam, th] of [
      [g, Math.atan2(H.x, -H.y)],
      [Math.PI - g, Math.atan2(-H.x, H.y)],
    ]) {
      const p = wrapAngle(beta - gam)
      let ab = wrapAngle(side * th)
      if (ab < -0.3) ab += 2 * Math.PI
      const cost =
        Math.abs(p - this.armP[i].x) + Math.abs(ab - this.armA[i].x) + (ab > 3.2 || p < -3 || p > 1.2 ? 10 : 0)
      if (cost < best) {
        best = cost
        this.armAim = [p, ab, el]
      }
    }
  }

  private solveLeg(H: Vector3, A: Vector3, pole: Vector3) {
    const L1 = DIM.thigh
    const L2 = DIM.shin
    const d = _v3a.subVectors(A, H)
    const raw = d.length() || 1e-6
    const dHat = d.divideScalar(raw)
    const dist = clamp(raw, 0.05, L1 + L2 - 1e-4)
    const a = (L1 * L1 - L2 * L2 + dist * dist) / (2 * dist)
    const hgt = Math.sqrt(Math.max(0, L1 * L1 - a * a))
    const pOrtho = _v3b.copy(pole).addScaledVector(dHat, -pole.dot(dHat)).normalize()
    _K.copy(H).addScaledVector(dHat, a).addScaledVector(pOrtho, hgt)
    const Aeff = _v3c.copy(H).addScaledVector(dHat, dist)

    const y = _v3d.subVectors(H, _K).normalize()
    const z = _pole.copy(pOrtho).addScaledVector(y, -pOrtho.dot(y)).normalize()
    const x = new Vector3().crossVectors(y, z)
    _m.makeBasis(x, y, z)
    _qThigh.setFromRotationMatrix(_m)

    y.subVectors(_K, Aeff).normalize()
    z.copy(pOrtho).addScaledVector(y, -pOrtho.dot(y)).normalize()
    x.crossVectors(y, z)
    _m.makeBasis(x, y, z)
    _qShin.setFromRotationMatrix(_m)
  }
}
