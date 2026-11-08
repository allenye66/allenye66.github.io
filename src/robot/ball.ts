import { Mesh, MeshPhysicalMaterial, Quaternion, SphereGeometry, Vector3 } from 'three'
import type { Object3D } from 'three'
import type { Part, Rig } from './model'

export const BALL_R = 0.11

const G = 9.81
const RESTITUTION_GROUND = 0.62
const RESTITUTION_BODY = 0.45

/** Segment with a radius, swept by a part of the robot. */
export interface Capsule {
  a: Vector3
  b: Vector3
  va: Vector3
  vb: Vector3
  r: number
  part: Part
  /** Leg index for leg capsules, else -1. */
  leg: number
}

const _d = new Vector3()
const _q = new Vector3()
const _vs = new Vector3()
const _ab = new Vector3()
const _axis = new Vector3()
const _dq = new Quaternion()

export class Ball {
  readonly mesh: Mesh
  readonly pos = new Vector3()
  readonly vel = new Vector3()
  readonly radius = BALL_R
  active = false
  /** Set by the scene: on screen and close enough to walk to. */
  reachable = false
  held = false
  readonly holdTarget = new Vector3()
  onHit: ((n: Vector3, speed: number, point: Vector3, part: Part) => void) | null = null

  private spin = new Vector3()
  private pop = 0
  private fading = false
  private opacity = 1
  private hitCooldown = 0
  private mat: MeshPhysicalMaterial

  constructor() {
    this.mat = new MeshPhysicalMaterial({
      color: 0xe6e6e3,
      roughness: 0.42,
      metalness: 0.05,
      clearcoat: 0.35,
      clearcoatRoughness: 0.3,
    })
    // Seams along three great circles, drawn in the shader so they stay crisp.
    this.mat.onBeforeCompile = (s) => {
      s.vertexShader = s.vertexShader
        .replace('#include <common>', '#include <common>\nvarying vec3 vBallP;')
        .replace('#include <begin_vertex>', '#include <begin_vertex>\nvBallP = position;')
      s.fragmentShader = s.fragmentShader
        .replace('#include <common>', '#include <common>\nvarying vec3 vBallP;')
        .replace(
          '#include <color_fragment>',
          `#include <color_fragment>
          vec3 bq = abs(normalize(vBallP));
          float bm = min(bq.x, min(bq.y, bq.z));
          float bw = fwidth(bm);
          float seam = 1.0 - smoothstep(0.03 - bw, 0.03 + bw, bm);
          diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.05), seam);`,
        )
    }
    this.mesh = new Mesh(new SphereGeometry(BALL_R, 48, 32), this.mat)
    this.mesh.castShadow = true
    this.mesh.receiveShadow = true
    this.mesh.visible = false
  }

  spawn(p: Vector3) {
    this.pos.copy(p)
    this.vel.set(0, 0, 0)
    this.spin.set(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5).multiplyScalar(6)
    this.mesh.quaternion.random()
    this.active = true
    this.held = false
    this.fading = false
    this.opacity = 1
    this.pop = 0
    this.setOpacity(1)
  }

  kick(v: Vector3) {
    this.vel.copy(v)
    // Backspin-free toe poke: tumble forward.
    _axis.set(v.z, 0, -v.x).normalize()
    this.spin.copy(_axis).multiplyScalar(v.length() / BALL_R * 0.6)
  }

  fadeOut() {
    if (this.active) this.fading = true
  }

  get opacityNow(): number {
    return this.fading ? Math.max(this.opacity, 0) : 1
  }

  get onGround(): boolean {
    return this.pos.y <= BALL_R + 0.002 && Math.abs(this.vel.y) < 0.05
  }

  step(h: number, caps: Capsule[], ghostLeg: number) {
    if (!this.active) return
    this.hitCooldown -= h
    const { pos, vel } = this

    if (this.held) {
      // Carried by the pointer: a stiff spring toward the target.
      _d.copy(this.holdTarget).sub(pos).multiplyScalar(500).addScaledVector(vel, -45)
      vel.addScaledVector(_d, h)
      pos.addScaledVector(vel, h)
      if (pos.y < BALL_R) {
        pos.y = BALL_R
        if (vel.y < 0) vel.y = 0
      }
      this.spin.multiplyScalar(Math.exp(-h * 2))
      this.collide(caps, ghostLeg, false)
      this.integrateSpin(h)
      return
    }

    vel.y -= G * h
    vel.multiplyScalar(Math.exp(-h * 0.08))
    pos.addScaledVector(vel, h)

    if (pos.y < BALL_R) {
      pos.y = BALL_R
      if (vel.y < 0) {
        const vy = -vel.y
        vel.y = vy > 0.45 ? vy * RESTITUTION_GROUND : 0
        // Contact friction trades some speed for spin.
        vel.x *= 0.93
        vel.z *= 0.93
      }
    }
    if (pos.y <= BALL_R + 1e-4 && vel.y === 0) {
      // Rolling: resistance slows it, and it spins to match.
      const sp = Math.hypot(vel.x, vel.z)
      if (sp > 0) {
        const ns = Math.max(0, sp - h * (0.35 + sp * 0.25))
        vel.x *= ns / sp
        vel.z *= ns / sp
      }
      this.spin.set(vel.z / BALL_R, 0, -vel.x / BALL_R)
    }

    this.collide(caps, ghostLeg, true)
    this.integrateSpin(h)
  }

  private collide(caps: Capsule[], ghostLeg: number, report: boolean) {
    const { pos, vel } = this
    for (const c of caps) {
      if (c.leg >= 0 && c.leg === ghostLeg) continue
      _ab.subVectors(c.b, c.a)
      const len2 = _ab.lengthSq()
      let u = len2 > 0 ? _d.subVectors(pos, c.a).dot(_ab) / len2 : 0
      u = u < 0 ? 0 : u > 1 ? 1 : u
      _q.copy(c.a).addScaledVector(_ab, u)
      _d.subVectors(pos, _q)
      const dist = _d.length()
      const min = BALL_R + c.r
      if (dist >= min || dist < 1e-6) continue
      _d.divideScalar(dist)
      pos.copy(_q).addScaledVector(_d, min)
      if (pos.y < BALL_R) pos.y = BALL_R
      _vs.copy(c.va).lerp(c.vb, u)
      const speed = vel.length()
      const vn = _d.dot(_ab.subVectors(vel, _vs))
      if (vn < 0) {
        vel.addScaledVector(_d, -(1 + RESTITUTION_BODY) * vn)
        if (report && this.onHit && -vn > 1.2 && speed > 1.5 && this.hitCooldown <= 0) {
          this.hitCooldown = 0.25
          this.onHit(_d.clone(), -vn, _q.clone(), c.part)
        }
      }
    }
  }

  private integrateSpin(h: number) {
    const w = this.spin.length()
    if (w > 1e-4) {
      _dq.setFromAxisAngle(_axis.copy(this.spin).divideScalar(w), w * h)
      this.mesh.quaternion.premultiply(_dq)
    }
  }

  /** Per frame: transform, spawn pop and fade. */
  sync(dt: number) {
    if (!this.active) {
      this.mesh.visible = false
      return
    }
    this.pop = Math.min(1, this.pop + dt / 0.16)
    if (this.fading) {
      this.opacity -= dt / 0.45
      if (this.opacity <= 0) {
        this.active = false
        this.held = false
        this.mesh.visible = false
        return
      }
      this.setOpacity(this.opacity)
    }
    const s = this.pop < 1 ? 1 - Math.pow(1 - this.pop, 3) * (1 - this.pop * 0.4) : 1
    this.mesh.visible = true
    this.mesh.position.copy(this.pos)
    this.mesh.scale.setScalar(Math.max(s, 0.001))
  }

  private setOpacity(o: number) {
    const fade = o < 1
    if (this.mat.transparent !== fade) {
      this.mat.transparent = fade
      this.mat.needsUpdate = true
    }
    this.mat.opacity = o
    this.mesh.castShadow = !fade
  }

  dispose() {
    this.mesh.geometry.dispose()
    this.mat.dispose()
  }
}

/** Capsules that approximate the robot's body for ball collisions. */
export class Body {
  readonly caps: Capsule[] = []
  private specs: { part: Part; leg: number; r: number; a: [Object3D, number, number, number]; b: [Object3D, number, number, number] }[] = []
  private first = true

  constructor(rig: Rig) {
    const add = (
      part: Part,
      leg: number,
      r: number,
      a: [Object3D, number, number, number],
      b: [Object3D, number, number, number],
    ) => {
      this.specs.push({ part, leg, r, a, b })
      this.caps.push({ a: new Vector3(), b: new Vector3(), va: new Vector3(), vb: new Vector3(), r, part, leg })
    }
    add('torso', -1, 0.125, [rig.root, 0, 0.02, 0], [rig.spine, 0, 0.3, 0])
    add('head', -1, 0.09, [rig.head, 0, 0.09, 0.004], [rig.head, 0, 0.14, 0.004])
    for (const i of [0, 1]) {
      add('arm', -1, 0.05, [rig.shoulder[i], 0, 0, 0], [rig.elbow[i], 0, 0, 0])
      add('arm', -1, 0.045, [rig.elbow[i], 0, 0, 0], [rig.wrist[i], 0, 0, 0])
      add('arm', -1, 0.04, [rig.wrist[i], 0, -0.03, 0], [rig.wrist[i], 0, -0.13, 0])
      add('leg', i, 0.075, [rig.hip[i], 0, 0, 0], [rig.knee[i], 0, 0, 0])
      add('leg', i, 0.055, [rig.knee[i], 0, 0, 0], [rig.ankle[i], 0, 0, 0])
      add('leg', i, 0.035, [rig.ankle[i], 0, -0.045, -0.04], [rig.ankle[i], 0, -0.045, 0.13])
    }
  }

  /** Call after the rig's world matrices are current. */
  update(dt: number) {
    for (let k = 0; k < this.specs.length; k++) {
      const s = this.specs[k]
      const c = this.caps[k]
      _q.copy(c.a)
      _d.copy(c.b)
      c.a.set(s.a[1], s.a[2], s.a[3]).applyMatrix4(s.a[0].matrixWorld)
      c.b.set(s.b[1], s.b[2], s.b[3]).applyMatrix4(s.b[0].matrixWorld)
      if (this.first || dt <= 0) {
        c.va.set(0, 0, 0)
        c.vb.set(0, 0, 0)
      } else {
        c.va.subVectors(c.a, _q).divideScalar(dt)
        c.vb.subVectors(c.b, _d).divideScalar(dt)
      }
    }
    this.first = false
  }
}
