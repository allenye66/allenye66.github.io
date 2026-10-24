---
name: dev
description: Run the site locally. Use when the user wants to preview, run, start, or work on the site locally, or see changes before deploying.
---

# Run allenye.me locally

1. If `node_modules/` is missing, or `package.json` is newer than it, run `npm install`.
2. Start the dev server in the background: `npm run dev`. It serves
   http://localhost:5173 with hot reload; edits to `src/` show up instantly.
   If port 5173 is taken, Vite picks the next free port and prints it; use that one.
3. Tell the user the URL. If browser tools are available and the user wants to
   see it, open the URL and take a screenshot.
4. To check the real production build instead (e.g. before a deploy):
   `npm run build && npm run preview` → http://localhost:4173.

When the user is done, stop the background dev server.
