import {
  CanvasTexture,
  DirectionalLight,
  HemisphereLight,
  Mesh,
  MeshBasicMaterial,
  NeutralToneMapping,
  PCFShadowMap,
  PerspectiveCamera,
  PlaneGeometry,
  PMREMGenerator,
  Raycaster,
  Scene,
  ShadowMaterial,
  SRGBColorSpace,
  Vector2,
  Vector3,
  WebGLRenderer,
} from 'three'
import type { Object3D } from 'three'
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js'
import { BALL_R, Ball, Body } from './ball'
import { Controller } from './controller'
import { buildRobot } from './model'
import type { Part } from './model'
import { clamp } from './math'

const FOV = 24
const HOME_YAW = -0.32
const SUBSTEP = 1 / 240
const POKE = 0.62
const BORED_AFTER = 15000

function blobTexture(): CanvasTexture {
  const c = document.createElement('canvas')
  c.width = c.height = 128
  const g = c.getContext('2d')!
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64)
  grad.addColorStop(0, 'rgba(255,255,255,1)')
  grad.addColorStop(0.45, 'rgba(255,255,255,0.55)')
  grad.addColorStop(1, 'rgba(255,255,255,0)')
  g.fillStyle = grad
  g.fillRect(0, 0, 128, 128)
  return new CanvasTexture(c)
}

export function mountRobot(container: HTMLElement): () => void {
  let renderer: WebGLRenderer
  try {
    renderer = new WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' })
  } catch {
    return () => {}
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = PCFShadowMap
  renderer.toneMapping = NeutralToneMapping
  renderer.outputColorSpace = SRGBColorSpace
  const canvas = renderer.domElement
  container.appendChild(canvas)

  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches
  const darkQuery = window.matchMedia('(prefers-color-scheme: dark)')

  const scene = new Scene()
  const pmrem = new PMREMGenerator(renderer)
  const room = new RoomEnvironment()
  const envRT = pmrem.fromScene(room, 0.04)
  scene.environment = envRT.texture

  const camera = new PerspectiveCamera(FOV, 1, 0.1, 80)

  const key = new DirectionalLight(0xffffff, 2)
  key.castShadow = true
  key.shadow.mapSize.set(2048, 2048)
  key.shadow.radius = 5
  key.shadow.bias = -0.0004
  key.shadow.normalBias = 0.012
  const sc = key.shadow.camera
  sc.left = -2.2
  sc.right = 2.2
  sc.top = 2.4
  sc.bottom = -2.4
  sc.near = 1
  sc.far = 20
  scene.add(key, key.target)
  const hemi = new HemisphereLight(0xffffff, 0xcfcfcf, 0.45)
  scene.add(hemi)
  // Rim light from behind so the black head and joints keep an edge.
  const rim = new DirectionalLight(0xffffff, 1.6)
  scene.add(rim, rim.target)

  const groundMat = new ShadowMaterial({ opacity: 0.12 })
  const ground = new Mesh(new PlaneGeometry(8, 8), groundMat)
  ground.rotation.x = -Math.PI / 2
  ground.receiveShadow = true
  scene.add(ground)

  // Soft contact shadows under each foot, the body and the ball.
  const blobTex = blobTexture()
  const blobGeo = new PlaneGeometry(1, 1)
  const blobs = [0, 1, 2, 3].map(() => {
    const m = new Mesh(
      blobGeo,
      new MeshBasicMaterial({ color: 0x000000, alphaMap: blobTex, transparent: true, depthWrite: false }),
    )
    m.rotation.x = -Math.PI / 2
    m.renderOrder = 1
    scene.add(m)
    return m
  })

  const rig = buildRobot()
  scene.add(rig.root)
  const ctl = new Controller(0, 0, HOME_YAW, reduced)
  const body = new Body(rig)
  const ball = new Ball()
  scene.add(ball.mesh)
  ctl.ball = ball
  ball.onHit = (n, speed, point, part) => ctl.hitByBall(n, speed, point, part)

  let blobBase = 0.2
  function applyTheme() {
    const dark = darkQuery.matches
    groundMat.opacity = dark ? 0.45 : 0.1
    blobBase = dark ? 0.6 : 0.2
    scene.environmentIntensity = dark ? 0.6 : 0.7
    key.intensity = dark ? 2.6 : 2.3
    hemi.intensity = dark ? 0.25 : 0.4
    rim.intensity = dark ? 2.4 : 1.4
  }
  applyTheme()
  darkQuery.addEventListener('change', applyTheme)

  // ------------------------------------------------------------ framing

  let metersPerPx = 0.003
  let viewH = 3
  function frame() {
    const w = container.clientWidth
    const h = container.clientHeight
    if (!w || !h) return
    renderer.setSize(w, h, false)
    const aspect = w / h
    camera.aspect = aspect
    const portrait = aspect < 0.9
    const frac = portrait ? 0.5 : 0.6
    const bottom = portrait ? 0.07 : 0.09
    const tanHalf = Math.tan(((FOV / 2) * Math.PI) / 180)
    viewH = 1.76 / frac
    const D = viewH / (2 * tanHalf)
    const elev = 0.07
    const alpha = Math.atan((1 - 2 * bottom) * tanHalf)
    const ty = D * Math.cos(elev) * Math.tan(alpha + elev) - D * Math.sin(elev)
    camera.position.set(0, ty + D * Math.sin(elev), D * Math.cos(elev))
    camera.lookAt(0, ty, 0)
    camera.updateProjectionMatrix()
    // Lens shift: on wide screens the robot stands right of the text column.
    const shift = clamp((aspect - 0.9) * 0.45, 0, 0.32)
    camera.projectionMatrix.elements[8] = -shift
    camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert()
    metersPerPx = viewH / h
    ctl.viewer.copy(camera.position)
  }
  frame()
  const ro = new ResizeObserver(frame)
  ro.observe(container)

  const _p = new Vector3()
  /** World point to client pixels. */
  function toScreen(v: Vector3, out: Vector2): Vector2 {
    _p.copy(v).project(camera)
    const r = canvas.getBoundingClientRect()
    return out.set(r.left + ((_p.x + 1) / 2) * r.width, r.top + ((1 - _p.y) / 2) * r.height)
  }

  // -------------------------------------------------------------- input

  const raycaster = new Raycaster()
  const ndc = new Vector2()
  let hasPointer = false
  let lastMove = -1e9
  let lastInput = performance.now()
  let boredAt = BORED_AFTER
  let hoverDirty = false

  function setNdc(x: number, y: number) {
    const r = canvas.getBoundingClientRect()
    ndc.set(((x - r.left) / r.width) * 2 - 1, -((y - r.top) / r.height) * 2 + 1)
  }

  function pick() {
    raycaster.setFromCamera(ndc, camera)
    const targets: Object3D[] = ball.active ? [...rig.meshes, ball.mesh] : rig.meshes
    return raycaster.intersectObjects(targets, false)[0] ?? null
  }

  function armIndex(o: Object3D): number {
    for (let p: Object3D | null = o; p; p = p.parent) {
      if (p === rig.shoulder[0]) return 0
      if (p === rig.shoulder[1]) return 1
    }
    return -1
  }

  interface Grab {
    id: number
    x0: number
    y0: number
    hit: Vector3
    part: Part
    arm: number
    moved: boolean
    samples: { x: number; y: number; t: number }[]
  }
  let grab: Grab | null = null

  interface BallGrab {
    id: number
    x0: number
    y0: number
    moved: boolean
    plane: Vector3
    samples: { p: Vector3; t: number }[]
  }
  let ballGrab: BallGrab | null = null

  const camRight = new Vector3()
  const camFwd = new Vector3()
  function cameraBasis() {
    camera.getWorldDirection(camFwd)
    camFwd.y = 0
    camFwd.normalize()
    camRight.set(-camFwd.z, 0, camFwd.x)
  }

  /** Where the pointer ray meets the upright plane through `at` facing the camera. */
  function rayToPlane(at: Vector3, out: Vector3): Vector3 {
    raycaster.setFromCamera(ndc, camera)
    cameraBasis()
    const o = raycaster.ray.origin
    const d = raycaster.ray.direction
    const t = _p.copy(at).sub(o).dot(camFwd) / d.dot(camFwd)
    return out.copy(o).addScaledVector(d, t)
  }

  function noteInput(now: number) {
    lastInput = now
    boredAt = BORED_AFTER
  }

  function onMove(e: PointerEvent) {
    const now = performance.now()
    hasPointer = true
    lastMove = now
    noteInput(now)
    setNdc(e.clientX, e.clientY)
    hoverDirty = true

    if (grab && e.pointerId === grab.id) {
      const dx = e.clientX - grab.x0
      const dy = e.clientY - grab.y0
      grab.samples.push({ x: e.clientX, y: e.clientY, t: now })
      if (grab.samples.length > 12) grab.samples.shift()
      if (!grab.moved && Math.hypot(dx, dy) > 6) {
        grab.moved = true
        ctl.startDrag(grab.hit, grab.part === 'arm' ? grab.arm : -1)
      }
      if (grab.moved) {
        cameraBasis()
        const target = grab.hit
          .clone()
          .addScaledVector(camRight, dx * metersPerPx)
          .addScaledVector(camFwd, -dy * metersPerPx * 1.6)
        target.y = grab.hit.y - dy * metersPerPx * 0.5
        target.x = clamp(target.x, -6, 6)
        target.z = clamp(target.z, -5, camera.position.z - 4)
        ctl.updateDrag(target)
      }
    }

    if (ballGrab && e.pointerId === ballGrab.id) {
      if (!ballGrab.moved && Math.hypot(e.clientX - ballGrab.x0, e.clientY - ballGrab.y0) > 6) {
        ballGrab.moved = true
        ball.held = true
      }
      if (ballGrab.moved) {
        const t = rayToPlane(ballGrab.plane, ball.holdTarget)
        t.y = clamp(t.y, BALL_R, 3)
        t.x = clamp(t.x, ctl.home.x - 6, ctl.home.x + 6)
        ballGrab.samples.push({ p: t.clone(), t: now })
        if (ballGrab.samples.length > 10) ballGrab.samples.shift()
      }
    }
  }

  let lastTap: { x: number; y: number; t: number } | null = null

  function dropBall() {
    // Drop from the pointer, on the upright plane through the robot.
    const p = rayToPlane(new Vector3(ctl.c.x, 0, ctl.c.y), new Vector3())
    if (p.y < 0.3) {
      // Below its feet on screen: drop onto the floor in front instead.
      const o = raycaster.ray.origin
      const d = raycaster.ray.direction
      if (d.y < -1e-3) p.copy(o).addScaledVector(d, -o.y / d.y)
      p.y = 0.3
    }
    p.y = Math.min(p.y, 2.8)
    p.x = clamp(p.x, ctl.home.x - 4.5, ctl.home.x + 4.5)
    p.z = clamp(p.z, ctl.home.y - 3, camera.position.z - 3.5)
    ball.spawn(p)
  }

  function onDown(e: PointerEvent) {
    const now = performance.now()
    noteInput(now)
    setNdc(e.clientX, e.clientY)
    const hit = pick()

    if (!hit) {
      // Double-tap empty space to drop a ball.
      if (lastTap && now - lastTap.t < 350 && Math.hypot(e.clientX - lastTap.x, e.clientY - lastTap.y) < 40) {
        lastTap = null
        dropBall()
      } else lastTap = { x: e.clientX, y: e.clientY, t: now }
      return
    }
    e.preventDefault()
    canvas.setPointerCapture(e.pointerId)
    canvas.style.cursor = 'grabbing'

    if (hit.object === ball.mesh) {
      ballGrab = {
        id: e.pointerId,
        x0: e.clientX,
        y0: e.clientY,
        moved: false,
        plane: ball.pos.clone(),
        samples: [],
      }
      return
    }
    grab = {
      id: e.pointerId,
      x0: e.clientX,
      y0: e.clientY,
      hit: hit.point.clone(),
      part: hit.object.userData.part as Part,
      arm: armIndex(hit.object),
      moved: false,
      samples: [{ x: e.clientX, y: e.clientY, t: now }],
    }
  }

  function onUp(e: PointerEvent) {
    if (ballGrab && e.pointerId === ballGrab.id) {
      const g = ballGrab
      ballGrab = null
      canvas.style.cursor = ''
      hoverDirty = true
      if (!g.moved) {
        // Tap the ball: nudge it away along the pointer ray.
        raycaster.setFromCamera(ndc, camera)
        const d = raycaster.ray.direction
        const len = Math.hypot(d.x, d.z) || 1
        ball.vel.x += (d.x / len) * 2.2
        ball.vel.z += (d.z / len) * 2.2
        ball.vel.y += 1.6
        return
      }
      ball.held = false
      const now = performance.now()
      const recent = g.samples.filter((s) => now - s.t < 100)
      if (recent.length >= 2) {
        const a = recent[0]
        const b = recent[recent.length - 1]
        const dt = Math.max((b.t - a.t) / 1000, 0.016)
        ball.vel.copy(b.p).sub(a.p).divideScalar(dt)
        if (ball.vel.length() > 9) ball.vel.setLength(9)
      } else ball.vel.set(0, 0, 0)
      return
    }

    if (!grab || e.pointerId !== grab.id) return
    const g = grab
    grab = null
    canvas.style.cursor = ''
    hoverDirty = true
    if (!g.moved) {
      raycaster.setFromCamera(ndc, camera)
      const dir = new Vector2(raycaster.ray.direction.x, raycaster.ray.direction.z)
      ctl.poke(dir, POKE * (0.9 + Math.random() * 0.2), g.hit, g.part)
      return
    }
    // Fling with the pointer's recent velocity.
    const now = performance.now()
    const recent = g.samples.filter((s) => now - s.t < 90)
    const fling = new Vector2()
    if (recent.length >= 2) {
      const a = recent[0]
      const b = recent[recent.length - 1]
      const dt = Math.max((b.t - a.t) / 1000, 0.016)
      cameraBasis()
      const vx = ((b.x - a.x) / dt) * metersPerPx
      const vy = ((b.y - a.y) / dt) * metersPerPx
      fling.set(camRight.x * vx - camFwd.x * vy * 1.6, camRight.z * vx - camFwd.z * vy * 1.6).multiplyScalar(0.4)
      if (fling.length() > 2) fling.setLength(2)
    }
    ctl.endDrag(fling)
  }

  window.addEventListener('pointermove', onMove)
  canvas.addEventListener('pointerdown', onDown)
  canvas.addEventListener('pointerup', onUp)
  canvas.addEventListener('pointercancel', onUp)

  // --------------------------------------------------------------- look

  const look = new Vector3()
  const wander = new Vector3()
  let wanderTimer = 0
  function updateLook(now: number, dt: number) {
    const idle = !hasPointer || now - lastMove > 3500
    const ballMoving = ball.active && (ball.held || ball.vel.length() > 0.3)
    if (grab?.moved) {
      raycaster.setFromCamera(ndc, camera)
      look.copy(raycaster.ray.origin).addScaledVector(raycaster.ray.direction, camera.position.z - 1.2)
    } else if (ball.active && (ballMoving || ctl.chasing)) {
      look.copy(ball.pos)
    } else if (!idle) {
      raycaster.setFromCamera(ndc, camera)
      look.copy(raycaster.ray.origin).addScaledVector(raycaster.ray.direction, camera.position.z - 2.2)
    } else {
      wanderTimer -= dt
      if (wanderTimer <= 0) {
        wanderTimer = 2.5 + Math.random() * 3
        const atViewer = reduced || Math.random() < 0.45
        wander.copy(camera.position)
        if (!atViewer) {
          wander.x += (Math.random() * 2 - 1) * 2.2
          wander.y += (Math.random() * 2 - 1.2) * 0.7
        }
      }
      look.copy(wander)
    }
    ctl.lookTarget.lerp(look, 1 - Math.exp(-dt * 10))
  }

  // --------------------------------------------------------------- ball

  const _ndc = new Vector3()
  let unreachable = 0
  function updateBall(dt: number) {
    if (!ball.active) {
      ball.reachable = false
      return
    }
    const p = ball.pos
    _ndc.copy(p).project(camera)
    const dHome = Math.hypot(p.x - ctl.home.x, p.z - ctl.home.y)
    const onScreen = Math.abs(_ndc.x) < 0.95 && _ndc.y > -1.05 && _ndc.y < 1.2
    ball.reachable = onScreen && dHome < 4.2 && p.z < camera.position.z - 3.2
    if (ball.held) {
      unreachable = 0
      return
    }
    unreachable = ball.reachable ? 0 : unreachable + dt
    const gone = Math.abs(_ndc.x) > 1.2 || _ndc.y < -1.25 || dHome > 10 || p.z > camera.position.z - 1.2
    if (gone || unreachable > 10) ball.fadeOut()
  }

  // --------------------------------------------------------------- loop

  let raf = 0
  let last = performance.now()
  let acc = 0
  let shown = false

  function tick(now: number) {
    raf = requestAnimationFrame(tick)
    const dt = Math.min((now - last) / 1000, 1 / 20)
    last = now
    acc += dt
    while (acc >= SUBSTEP) {
      ctl.step(SUBSTEP)
      ball.step(SUBSTEP, body.caps, ctl.ghostLeg)
      acc -= SUBSTEP
    }
    updateBall(dt)
    updateLook(now, dt)
    ctl.apply(rig, dt)
    body.update(dt)
    ball.sync(dt)

    if (now - lastInput > boredAt && ctl.bored()) boredAt += 15000 + Math.random() * 10000

    for (let i = 0; i < 2; i++) {
      const f = ctl.feet[i]
      const b = blobs[i]
      b.position.set(f.pos.x + Math.sin(f.yaw) * 0.04, 0.001, f.pos.y + Math.cos(f.yaw) * 0.04)
      b.rotation.z = f.yaw
      b.scale.set(0.2, 0.36, 1)
      ;(b.material as MeshBasicMaterial).opacity = blobBase * clamp(1 - f.lift * 9, 0.15, 1)
    }
    const bodyBlob = blobs[2]
    bodyBlob.position.set(ctl.c.x, 0.0005, ctl.c.y)
    bodyBlob.scale.set(0.6, 0.42, 1)
    ;(bodyBlob.material as MeshBasicMaterial).opacity = blobBase * 0.3
    const ballBlob = blobs[3]
    ballBlob.visible = ball.mesh.visible
    if (ballBlob.visible) {
      const y = ball.pos.y - BALL_R
      ballBlob.position.set(ball.pos.x, 0.0008, ball.pos.z)
      ballBlob.scale.setScalar(0.3 * (1 + y * 0.5) * ball.mesh.scale.x)
      ;(ballBlob.material as MeshBasicMaterial).opacity = blobBase * 0.9 * clamp(1 - y * 1.1, 0.12, 1) * ball.opacityNow
    }

    ground.position.set(ctl.c.x, 0, ctl.c.y)
    key.position.set(ctl.c.x - 1.1, 7, ctl.c.y + 1.9)
    key.target.position.set(ctl.c.x, 0.8, ctl.c.y)
    rim.position.set(ctl.c.x + 2.5, 3.5, ctl.c.y - 4)
    rim.target.position.set(ctl.c.x, 1, ctl.c.y)

    if (hoverDirty && !grab && !ballGrab) {
      hoverDirty = false
      canvas.style.cursor = pick() ? 'grab' : ''
    }

    renderer.render(scene, camera)
    if (!shown) {
      shown = true
      container.classList.add('ready')
    }
  }
  raf = requestAnimationFrame(tick)

  if (import.meta.env.DEV) {
    // Dev-only handle for poking at the simulation from the console.
    const w = window as unknown as Record<string, unknown>
    w.__robot = {
      ctl,
      rig,
      ball,
      camera,
      dropBall(x: number, y: number) {
        setNdc(x, y)
        dropBall()
      },
      screenOf(o: Object3D | Vector3) {
        const v = o instanceof Vector3 ? o : o.getWorldPosition(new Vector3())
        const s = toScreen(v, new Vector2())
        return { x: s.x, y: s.y }
      },
    }
  }

  return () => {
    cancelAnimationFrame(raf)
    ro.disconnect()
    darkQuery.removeEventListener('change', applyTheme)
    window.removeEventListener('pointermove', onMove)
    canvas.removeEventListener('pointerdown', onDown)
    canvas.removeEventListener('pointerup', onUp)
    canvas.removeEventListener('pointercancel', onUp)
    rig.dispose()
    ball.dispose()
    blobGeo.dispose()
    blobTex.dispose()
    for (const b of blobs) (b.material as MeshBasicMaterial).dispose()
    ground.geometry.dispose()
    groundMat.dispose()
    envRT.dispose()
    pmrem.dispose()
    room.dispose()
    renderer.dispose()
    canvas.remove()
    container.classList.remove('ready')
  }
}
