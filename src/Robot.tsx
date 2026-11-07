import { useEffect, useRef } from 'react'

// The 3D robot is loaded after first paint so the text never waits on it.
export default function Robot() {
  const ref = useRef<HTMLDivElement>(null)

  useEffect(() => {
    let dispose: (() => void) | undefined
    let cancelled = false
    import('./robot/index')
      .then(({ mountRobot }) => {
        if (!cancelled && ref.current) dispose = mountRobot(ref.current)
      })
      .catch(() => {})
    return () => {
      cancelled = true
      dispose?.()
    }
  }, [])

  return <div ref={ref} className="robot" aria-hidden="true" />
}
