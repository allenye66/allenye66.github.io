import { clamp, damp, smoothstep } from './math'

// Upper-body gestures layered over the balance controller's arm and head
// motion. A gesture sets pose targets; weights fade so it blends in and out.

export type PalmAim = 'head' | 'back' | null
export type HeadLook = 'viewer' | 'hand' | 'ahead' | null

export interface ArmPose {
  /** Shoulder pitch (negative raises forward) and abduction (positive outward). */
  p: number
  a: number
  /** Twist about the upper arm. */
  tw: number
  el: number
  /** Wrist pitch. */
  wp: number
  /** Where the palm turns to face, by rolling the wrist. */
  palm: PalmAim
  /** Finger curl, index to pinky: 0 relaxed, 1 fist, negative opens. */
  curl: [number, number, number, number]
}

type Kind = 'inspect' | 'neckroll' | 'shrug' | 'glance'

interface Step {
  kind: Kind
  arm: number
  dur: number
  /** Random side for head tilts. */
  tilt: number
}

function makeArm(): ArmPose {
  return { p: 0, a: 0.1, tw: 0, el: -0.22, wp: 0, palm: null, curl: [0, 0, 0, 0] }
}

/** Smooth 0 → 1 → 0 over x in [0, 1]. */
function bump(x: number): number {
  return x <= 0 || x >= 1 ? 0 : Math.sin(Math.PI * x)
}

export class Gestures {
  readonly arm: [ArmPose, ArmPose] = [makeArm(), makeArm()]
  /** Smoothed blend weight per arm. */
  readonly armW = [0, 0]
  look: HeadLook = null
  lookArm = 0
  lookYaw = 0
  lookPitch = 0
  /** Smoothed weight of the head override. */
  lookW = 0
  roll = 0
  shrug = 0

  private queue: Step[] = []
  private cur: Step | null = null
  private t = 0
  private boredCount = 0
  private armTarget = [0, 0]
  private lookTarget = 0
  private rollTarget = 0
  private shrugTarget = 0

  get active(): boolean {
    return this.cur !== null
  }

  bored() {
    const tilt = Math.random() < 0.5 ? 1 : -1
    if (this.boredCount++ % 2 === 0) {
      const arm = Math.random() < 0.5 ? 0 : 1
      this.play([
        { kind: 'inspect', arm, dur: 4.8, tilt },
        { kind: 'glance', arm, dur: 2.2, tilt },
      ])
    } else {
      this.play([
        { kind: 'neckroll', arm: 0, dur: 2.9, tilt },
        { kind: 'shrug', arm: 0, dur: 1.1, tilt },
        { kind: 'glance', arm: 0, dur: 2.2, tilt: -tilt },
      ])
    }
  }

  cancel() {
    this.queue = []
    this.cur = null
  }

  private play(steps: Step[]) {
    this.queue = steps.slice(1)
    this.cur = steps[0]
    this.t = 0
  }

  update(h: number) {
    if (this.cur) {
      this.t += h
      if (this.t >= this.cur.dur) {
        this.cur = this.queue.shift() ?? null
        this.t = 0
      }
    }
    this.armTarget[0] = this.armTarget[1] = 0
    this.lookTarget = 0
    this.rollTarget = 0
    this.shrugTarget = 0
    if (this.cur) this.evaluate(this.cur, this.t)
    for (const i of [0, 1]) {
      const rate = this.armTarget[i] > this.armW[i] ? 7 : 4.5
      this.armW[i] = damp(this.armW[i], this.armTarget[i], rate, h)
    }
    this.lookW = damp(this.lookW, this.lookTarget, this.lookTarget > this.lookW ? 6 : 4, h)
    this.roll = damp(this.roll, this.rollTarget, 6, h)
    this.shrug = damp(this.shrug, this.shrugTarget, 9, h)
  }

  private evaluate(s: Step, t: number) {
    const side = s.arm === 0 ? 1 : -1
    const A = this.arm[s.arm]
    const end = t < s.dur - 0.45 ? 1 : 0
    switch (s.kind) {
      case 'inspect': {
        // Bring a hand up and study it: flex the fingers, turn it over, make a fist.
        A.p = -0.8
        A.a = -0.04
        A.tw = -side * 0.45
        A.el = -1.45
        A.wp = 0.1
        A.palm = t > 2.2 && t < 2.95 ? 'back' : 'head'
        for (let f = 0; f < 4; f++) {
          const ripple = 0.95 * bump((t - 0.8 - f * 0.13) / 0.55)
          const fist = 0.92 * bump((t - 3.05 - f * 0.04) / 1.0)
          A.curl[f] = Math.max(ripple, fist) - 0.05
        }
        this.armTarget[s.arm] = t < s.dur - 0.6 ? 1 : 0
        this.look = 'hand'
        this.lookArm = s.arm
        this.lookYaw = 0
        this.lookPitch = 0.18
        this.lookTarget = t < s.dur - 0.5 ? 1 : 0
        this.rollTarget = side * 0.12
        break
      }
      case 'neckroll': {
        // Chin down, ear to shoulder, head back, other ear.
        const phi = 2 * Math.PI * clamp((t - 0.25) / 2.3, 0, 1) * s.tilt
        const env = smoothstep(t / 0.45) * (1 - smoothstep((t - s.dur + 0.5) / 0.45))
        this.look = 'ahead'
        this.lookYaw = 0.22 * Math.sin(phi) * env
        this.lookPitch = 0.3 * Math.cos(phi) * env
        this.lookTarget = 1
        this.rollTarget = 0.3 * Math.sin(phi) * env
        break
      }
      case 'shrug': {
        this.shrugTarget = bump(t / (s.dur - 0.2))
        this.look = 'ahead'
        this.lookYaw = 0
        this.lookPitch = -0.08 * bump(t / s.dur)
        this.lookTarget = 0.6
        break
      }
      case 'glance': {
        // A curious look at the viewer, head tilted.
        this.look = 'viewer'
        this.lookYaw = 0
        this.lookPitch = 0
        this.lookTarget = end
        this.rollTarget = s.tilt * 0.16 * smoothstep(t / 0.5) * end
        break
      }
    }
  }
}
