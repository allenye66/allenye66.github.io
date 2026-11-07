import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  // three.js ships in its own lazy chunk (src/robot), loaded after first paint.
  build: { chunkSizeWarningLimit: 700 },
})
