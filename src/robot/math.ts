import { Vector2 } from 'three'

// Ground-plane vectors use Vector2(x, y) where y is world z.

export function fwd(yaw: number, out = new Vector2()): Vector2 {
  return out.set(Math.sin(yaw), Math.cos(yaw))
}

export function left(yaw: number, out = new Vector2()): Vector2 {
  return out.set(Math.cos(yaw), -Math.sin(yaw))
}

export function wrapAngle(a: number): number {
  while (a > Math.PI) a -= 2 * Math.PI
  while (a < -Math.PI) a += 2 * Math.PI
  return a
}

export function lerpAngle(a: number, b: number, t: number): number {
  return a + wrapAngle(b - a) * t
}

export function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x
}

/** Frame-rate independent exponential approach. */
export function damp(a: number, b: number, lambda: number, dt: number): number {
  return b + (a - b) * Math.exp(-lambda * dt)
}

export function smoothstep(t: number): number {
  t = clamp(t, 0, 1)
  return t * t * (3 - 2 * t)
}

function cross(o: Vector2, a: Vector2, b: Vector2): number {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/** Andrew's monotone chain. Returns a counter-clockwise hull. */
export function convexHull(pts: Vector2[]): Vector2[] {
  const p = pts.slice().sort((a, b) => a.x - b.x || a.y - b.y)
  if (p.length < 3) return p
  const lower: Vector2[] = []
  for (const q of p) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], q) <= 0) lower.pop()
    lower.push(q)
  }
  const upper: Vector2[] = []
  for (let i = p.length - 1; i >= 0; i--) {
    const q = p[i]
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], q) <= 0) upper.pop()
    upper.push(q)
  }
  upper.pop()
  lower.pop()
  return lower.concat(upper)
}

const tmpA = new Vector2()

/**
 * Writes the closest point of a convex CCW polygon to `q` into `out` and
 * returns the distance from q to the polygon (0 when inside).
 */
export function clampToPoly(q: Vector2, poly: Vector2[], out: Vector2): number {
  const n = poly.length
  let inside = true
  for (let i = 0; i < n; i++) {
    if (cross(poly[i], poly[(i + 1) % n], q) < 0) {
      inside = false
      break
    }
  }
  if (inside) {
    out.copy(q)
    return 0
  }
  let best = Infinity
  for (let i = 0; i < n; i++) {
    const a = poly[i]
    const b = poly[(i + 1) % n]
    const abx = b.x - a.x
    const aby = b.y - a.y
    const len2 = abx * abx + aby * aby
    const t = len2 > 0 ? clamp(((q.x - a.x) * abx + (q.y - a.y) * aby) / len2, 0, 1) : 0
    tmpA.set(a.x + abx * t, a.y + aby * t)
    const d = tmpA.distanceTo(q)
    if (d < best) {
      best = d
      out.copy(tmpA)
    }
  }
  return best
}

/** Damped spring integrated with semi-implicit Euler. */
export class Spring {
  x: number
  v = 0
  constructor(x = 0) {
    this.x = x
  }
  step(target: number, k: number, d: number, dt: number) {
    this.v += (k * (target - this.x) - d * this.v) * dt
    this.x += this.v * dt
  }
}
