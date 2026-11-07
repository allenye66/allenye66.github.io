import { Euler, Matrix4, Quaternion, Vector2, Vector3 } from 'three'
import { DIM } from './model'
import type { Part, Rig } from './model'
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
const DRAG_K = 26
const DRAG_D = 2.5
const EXT_MAX = 9

type StepKind = 'capture' | 'walk' | 'tidy'

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
  pitch: number
}

interface Drag {
  target: Vector2
  offset: Vector2
  height: number
  arm: number
  aim: Vector3
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
    pitch: 0,
  }
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

const _v3a = new Vector3()
const _v3b = new Vector3()
const _v3c = new Vector3()
const _v3d = new Vector3()
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
const _e = new Euler()

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
  private mode: 'stand' | 'walk' = 'stand'
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

  private aFilt = new Vector2()
  private vSlow = new Vector2()

  private leanP = new Spring()
  private leanR = new Spring()
  private twist = new Spring()
  private headYaw = new Spring()
  private headPitch = new Spring()
  private armP = [new Spring(0.04), new Spring(0.04)]
  private armA = [new Spring(0.1), new Spring(0.1)]
  private armE = [new Spring(-0.22), new Spring(-0.22)]
  private armAim: [number, number] | null = null

  private pelvisH = DIM.standPelvis
  private pelvisYaw: number

  private shift = 0
  private shiftTarget = 0
  private shiftTimer = 6

  lookTarget = new Vector3(0, 1.6, 5)

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

  startDrag(hit: Vector3, arm: number) {
    this.drag = {
      target: new Vector2(hit.x, hit.z),
      offset: new Vector2(hit.x - this.c.x, hit.z - this.c.y),
      height: hit.y,
      arm,
      aim: hit.clone(),
    }
    this.interrupt()
  }

  updateDrag(target: Vector3) {
    if (!this.drag) return
    this.drag.target.set(target.x, target.z)
    this.drag.aim.copy(target)
  }

  endDrag(fling: Vector2) {
    this.v.add(fling)
    this.drag = null
    this.armAim = null
    this.interrupt()
  }

  private interrupt() {
    this.mode = 'stand'
    this.heading = this.homeYaw
    this.tripFace = null
    this.walkLaunched = false
    this.tidyPhase = 'none'
    this.settledTime = 0
    if (this.swing >= 0 && this.stepKind !== 'capture') this.stepKind = 'capture'
  }

  // ------------------------------------------------------------- geometry

  private footCenter(f: Foot, out: Vector2): Vector2 {
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

  // ------------------------------------------------------------ simulation

  step(h: number) {
    this.time += h
    const { c, v } = this

    this.ext.set(0, 0)
    if (this.drag) {
      _a.copy(c).add(this.drag.offset)
      this.ext.copy(this.drag.target).sub(_a).multiplyScalar(DRAG_K).addScaledVector(v, -DRAG_D)
      if (this.ext.length() > EXT_MAX) this.ext.setLength(EXT_MAX)
    }

    const xi = _xi.copy(v).divideScalar(W).add(c)
    const xiE = _xiE.copy(this.ext).divideScalar(W2).add(xi)

    let outside = 0
    let gap = 0
    if (this.swing < 0) {
      const poly = this.supportPoly()
      const s = this.standTarget(h)
      _pDes.copy(xi).sub(s).multiplyScalar(K_XI / W).add(xiE)
      clampToPoly(_pDes, poly, this.p)
      outside = clampToPoly(xiE, poly, _out)
      gap = outside - 0.5
    } else {
      const st = this.feet[1 - this.swing]
      const poly = this.footPoly(st)
      if (this.stepKind === 'walk') {
        this.footCenter(st, this.p)
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
    if (this.swing >= 0) {
      const sw = this.feet[this.swing]
      sw.t = Math.min(1, sw.t + h / sw.dur)
      if (this.stepKind !== 'tidy' && sw.t < 0.72) {
        const tgt = this.planTarget(this.swing, sw.dur * (1 - sw.t), this.stepKind)
        sw.to.lerp(tgt, 1 - Math.exp(-h * 16))
      }
      const e = smoothstep(sw.t)
      sw.pos.copy(sw.from).lerp(sw.to, e)
      sw.yaw = lerpAngle(sw.fromYaw, sw.toYaw, e)
      const stride = clamp(sw.from.distanceTo(sw.to) / 0.35, 0.25, 1)
      sw.lift = sw.height * Math.pow(Math.sin(Math.PI * sw.t), 0.85)
      sw.pitch = 0.3 * stride * Math.sin(2 * Math.PI * sw.t)
      if (sw.t >= 1) this.touchdown()
      return
    }

    this.dsTime += h
    const settled = outside === 0 && this.v.length() < 0.07 && !this.drag
    this.settledTime = settled ? this.settledTime + h : 0

    if (this.mode === 'walk') {
      const mid = this.mid(_a)
      const toHome = _b.copy(this.home).sub(mid)
      const dist = toHome.length()
      // Hand over to the balance controller near home; its last capture step
      // absorbs the remaining walking speed.
      const off = Math.abs(wrapAngle(Math.atan2(toHome.x, toHome.y) - this.heading))
      const astray = this.walkFace && off > 1.2 && off < Math.PI - 0.6
      // Watchdog: hand back to balance if the walk stops making progress.
      if (dist < this.walkBest - 0.05) {
        this.walkBest = dist
        this.walkStall = 0
      } else this.walkStall += h
      if (dist < 0.3 || astray || this.walkSteps > 30 || this.v.length() > 1.1 || this.walkStall > 2.5) {
        this.mode = 'stand'
      } else {
        toHome.divideScalar(dist)
        this.walkDir.lerp(toHome, 1 - Math.exp(-h * 4)).normalize()
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
      this.tidyTime += h
      const center = this.footCenter(this.feet[1 - this.tidySwing], _a)
      if ((xiE.distanceTo(center) < 0.03 && this.c.distanceTo(center) < 0.05) || this.tidyTime > 0.9) {
        this.tidyPhase = 'none'
        this.startStep(this.tidySwing, 'tidy', 0)
      }
    } else if (this.settledTime > 0.45 && !this.drag) {
      // Sideways walking is unstable for a biped. Far from home it turns in
      // place to face home and walks forward; a short hop straight forward or
      // back is walked while still facing the viewer. Near home it turns back.
      const mid = this.mid(_a)
      const d = _b.copy(this.home).sub(mid)
      const dist = d.length()
      d.divideScalar(dist || 1)
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
        if (pick >= 0) {
          this.tidyPhase = 'shift'
          this.tidySwing = pick
          this.tidyTime = 0
        }
      }
    }
  }

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
      // Strides shrink to zero near home, so the last steps brake to a stop.
      const dist = this.home.distanceTo(mid)
      const L0 = dist < 0.2 ? 0 : clamp(dist * 0.5, 0.08, maxL)
      target.addScaledVector(this.walkDir, -L0 / (Math.exp(W * WALK_T) - 1))
    }
    target.addScaledVector(L, sw.side * (kind === 'walk' ? 0.035 : W_OUT))

    const rx = target.x - st.pos.x
    const rz = target.y - st.pos.y
    let lat = (rx * L.x + rz * L.y) * sw.side
    let fw = rx * F.x + rz * F.y
    lat = Math.max(lat, MIN_LAT)
    const len = Math.hypot(lat, fw)
    if (len > MAX_STEP) {
      lat = Math.max(lat * (MAX_STEP / len), MIN_LAT)
      fw = Math.sign(fw) * Math.sqrt(Math.max(0, MAX_STEP * MAX_STEP - lat * lat))
    }
    return target
      .copy(st.pos)
      .addScaledVector(L, sw.side * lat)
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
    sw.dur = kind === 'capture' ? this.captureDur(urgency)  : kind === 'walk' ? WALK_T : 0.42
    sw.toYaw = this.planYaw(i)
    sw.to.copy(this.planTarget(i, sw.dur, kind))
    const dist = sw.to.distanceTo(sw.from)
    sw.height = kind === 'tidy' ? 0.035 + dist * 0.05 : clamp(0.045 + dist * 0.12, 0.05, 0.11)
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

    // Pelvis yaw follows the feet.
    let yawT: number
    if (this.swing < 0) yawT = lerpAngle(this.feet[0].yaw, this.feet[1].yaw, 0.5)
    else {
      const sw = this.feet[this.swing]
      const st = this.feet[1 - this.swing]
      yawT = lerpAngle(st.yaw, sw.toYaw, 0.5 * smoothstep(sw.t))
    }
    this.pelvisYaw = lerpAngle(this.pelvisYaw, yawT, 1 - Math.exp(-h * 8))

    // Upper body lags behind accelerations and leans into a pull.
    const hf = this.drag ? clamp((this.drag.height - COM_H) / 0.45, -1, 1) : 0
    _lean.copy(this.aFilt).multiplyScalar(-0.02).addScaledVector(this.ext, 0.028 * Math.max(hf, 0.3))
    if (this.mode === 'walk' && this.walkLaunched) _lean.addScaledVector(this.walkDir, 0.05)
    this.leanP.step(clamp(_lean.dot(F), -0.35, 0.35), 70, 11, h)
    this.leanR.step(clamp(-_lean.dot(L), -0.3, 0.3), 70, 11, h)
    this.twist.step(0, 45, 8, h)

    // Arms: counter-swing to the opposite leg, flare on sudden velocity changes.
    const hp = _hp.copy(this.v).sub(this.vSlow)
    const hpF = hp.dot(F)
    const hpL = hp.dot(L)
    const hpMag = Math.min(hp.length(), 1.5)
    for (const i of [0, 1]) {
      const side = this.feet[i].side
      const opp = this.feet[1 - i]
      const oppFwd = (opp.pos.x - this.c.x) * F.x + (opp.pos.y - this.c.y) * F.y + FOOT_CENTER
      let p = 0.04 - 1.3 * oppFwd + 1.2 * hpF
      let a = 0.1 + 1.1 * hpMag - side * 0.9 * hpL
      let el = -0.22 - 0.8 * hpMag
      if (this.drag && this.drag.arm === i && this.armAim) {
        ;[p, a] = this.armAim
        el = -0.08
      }
      this.armP[i].step(clamp(p, -2.4, 0.9), 90, 10, h)
      this.armA[i].step(clamp(a, 0.04, 1.6), 90, 10, h)
      this.armE[i].step(clamp(el, -1.6, -0.03), 110, 12, h)
    }

    // Head follows the look target, relative to the body.
    const t = this.lookTarget
    const rx = t.x - this.c.x
    const ry = t.y - 1.58
    const rz = t.z - this.c.y
    const lx = rx * L.x + rz * L.y
    const lz = rx * F.x + rz * F.y
    let hy = Math.atan2(lx, lz)
    if (Math.abs(hy) > 1.9) hy = 0
    const hpch = Math.atan2(-ry, Math.hypot(lx, lz))
    this.headYaw.step(clamp(hy, -1.1, 1.1) - this.twist.x, 55, 12, h)
    this.headPitch.step(clamp(hpch, -0.45, 0.5), 55, 12, h)
  }

  // ----------------------------------------------------------------- pose

  apply(rig: Rig, dt: number) {
    const yaw = this.pelvisYaw
    const L = left(yaw, _l)

    // Pelvis height: as tall as the planted legs allow, a little lower when
    // stepping urgently or walking.
    let maxH = Infinity
    for (const f of this.feet) {
      const hx = this.c.x + L.x * f.side * DIM.hipX
      const hz = this.c.y + L.y * f.side * DIM.hipX
      const dh = Math.hypot(f.pos.x - hx, f.pos.y - hz)
      const reach = dh < LEG_MAX ? Math.sqrt(LEG_MAX * LEG_MAX - dh * dh) : 0
      const lim = DIM.ankleH + f.lift + reach - DIM.hipY
      if (f.planted || f.t > 0.75) maxH = Math.min(maxH, lim)
    }
    const hpMag = Math.min(_hp.copy(this.v).sub(this.vSlow).length(), 1)
    const breath = this.reduced ? 0 : Math.sin(this.time * 1.5) * 0.0015
    const crouch = (this.mode === 'walk' && this.walkLaunched ? 0.015 : 0) + 0.035 * hpMag
    const want = Math.min(DIM.standPelvis - crouch + breath, maxH)
    this.pelvisH = Math.max(Math.min(damp(this.pelvisH, want, 20, dt), maxH), 0.74)

    let roll = this.leanR.x * 0.25
    if (this.swing >= 0) roll -= this.feet[this.swing].side * 0.035 * Math.sin(Math.PI * this.feet[this.swing].t)

    const root = rig.root
    root.position.set(this.c.x, this.pelvisH, this.c.y)
    _e.set(this.leanP.x * 0.25, yaw, roll, 'YXZ')
    root.quaternion.setFromEuler(_e)

    rig.spine.rotation.set(this.leanP.x * 0.75 + (this.reduced ? 0 : Math.sin(this.time * 1.5) * 0.006), this.twist.x, this.leanR.x * 0.75, 'YXZ')
    rig.neck.rotation.set(this.headPitch.x * 0.35, this.headYaw.x * 0.4, 0)
    rig.head.rotation.set(this.headPitch.x * 0.65, this.headYaw.x * 0.6, 0)

    for (const i of [0, 1]) {
      const side = this.feet[i].side
      rig.shoulder[i].rotation.set(this.armP[i].x, 0, side * this.armA[i].x)
      rig.elbow[i].rotation.set(this.armE[i].x, 0, 0)
      rig.wrist[i].rotation.set(-0.12, 0, 0)
    }

    root.updateMatrixWorld(true)
    _qRoot.copy(root.quaternion)
    _qRootInv.copy(_qRoot).invert()

    for (const i of [0, 1]) {
      const f = this.feet[i]
      rig.hip[i].getWorldPosition(_H)
      _A.set(f.pos.x, DIM.ankleH + f.lift, f.pos.y)
      _pole.set(Math.sin(f.yaw), 0, Math.cos(f.yaw))
      _pole.x += Math.sin(yaw) + L.x * f.side * 0.15
      _pole.z += Math.cos(yaw) + L.y * f.side * 0.15
      this.solveLeg(_H, _A, _pole)
      rig.hip[i].quaternion.copy(_qRootInv).multiply(_qThigh)
      rig.knee[i].quaternion.copy(_qTmp.copy(_qThigh).invert()).multiply(_qShin)
      _e.set(f.pitch, f.yaw, 0, 'YXZ')
      _qFoot.setFromEuler(_e)
      rig.ankle[i].quaternion.copy(_qTmp.copy(_qShin).invert()).multiply(_qFoot)
    }

    // Aim a grabbed arm at the pointer, in the spine's frame.
    if (this.drag && this.drag.arm >= 0) {
      const sh = rig.shoulder[this.drag.arm]
      sh.getWorldPosition(_v3a)
      const dir = _v3b.copy(this.drag.aim).sub(_v3a)
      if (dir.lengthSq() > 1e-6) {
        rig.spine.getWorldQuaternion(_qTmp)
        dir.applyQuaternion(_qTmp.invert()).normalize()
        const p = -Math.asin(clamp(dir.z, -1, 1))
        const side = this.feet[this.drag.arm].side
        const r = Math.atan2(dir.x, -dir.y)
        this.armAim = [p, side * r]
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
