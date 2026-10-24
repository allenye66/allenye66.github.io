# AGENTS.md

Allen Ye's personal site, live at https://allenye.me.

## Stack

- Vite + React 19 + TypeScript. No router, no UI library, no CSS framework.
- Hosted on GitHub Pages from the `gh-pages` branch of `allenye66/allenye66.github.io`.
- Domain `allenye.me` is registered at Namecheap. DNS: 4 A records on `@` to
  `185.199.108-111.153`, `www` CNAME to `allenye66.github.io`, and a
  `_github-pages-challenge-allenye66` TXT record that keeps the domain verified
  on Allen's GitHub account. Don't remove the TXT record.

## Layout

```
src/App.tsx       page content (name, tagline, links array)
src/index.css     all styles; colors are CSS vars on :root with a dark-mode override
src/main.tsx      React entry, rarely touched
index.html        HTML shell: <title>, meta description
public/           copied as-is into the build (CNAME, images, PDFs)
public/CNAME      must stay "allenye.me" or the custom domain detaches on deploy
scripts/          helper scripts (check-live.sh)
```

## Commands

| Task | Command |
|---|---|
| Install deps | `npm install` |
| Local dev server (hot reload) | `npm run dev` → http://localhost:5173 |
| Type-check + production build | `npm run build` → `dist/` |
| Preview the production build | `npm run preview` |
| Lint | `npm run lint` |
| Deploy to allenye.me | `npm run deploy` |
| Confirm the deploy is live | `./scripts/check-live.sh` |

## How deploys work

`npm run deploy` runs `predeploy` (`npm run build`), then
`gh-pages -d dist --nojekyll`, which force-pushes `dist/` to the `gh-pages`
branch. GitHub Pages rebuilds in about 30–90s.

- Pushing to `master` does **not** deploy. `master` is source only.
- Deploy publishes whatever is in the working tree, committed or not.
  Commit first so `master` matches what's live.
- Don't edit the `gh-pages` branch by hand; every deploy overwrites it.

## Conventions

- Keep the site minimal: small type, lots of whitespace, no animations or
  trackers unless Allen asks.
- Content lives in `src/App.tsx`. Links go in the `links` array.
- Static files (resume PDF, images) go in `public/` and are referenced by
  absolute path, e.g. `/resume.pdf`.
- Respect both light and dark mode: add colors as vars in `src/index.css`.
- Run `npm run build` before deploying; it fails on type errors.

## Skills

Claude Code skills in `.claude/skills/`:

- `deploy`: build, commit, deploy, and verify the site is live.
- `dev`: start the local dev server and open it.
- `site-status`: check DNS, HTTPS, and Pages health when something looks broken.
