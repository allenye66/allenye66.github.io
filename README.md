# allenye.me

My personal site: https://allenye.me

Vite + React + TypeScript, hosted on GitHub Pages.

## Quick start

```sh
npm install      # once, or after pulling dependency changes
npm run dev      # http://localhost:5173, reloads as you edit
```

Edit `src/App.tsx` for content and `src/index.css` for styles.

## Deploy

```sh
git add -A && git commit -m "..." && git push   # save source (does NOT deploy)
npm run deploy                                    # build + publish to allenye.me
./scripts/check-live.sh                           # wait until the new build is live
```

`npm run deploy` builds into `dist/` and pushes it to the `gh-pages` branch.
GitHub Pages serves that branch, usually within a minute.

In Claude Code you can just say "deploy" (or run `/deploy`) and it will do all of the above.

## All commands

| Command | What it does |
|---|---|
| `npm run dev` | Local dev server with hot reload |
| `npm run build` | Type-check and build to `dist/` |
| `npm run preview` | Serve the built `dist/` locally to check the production build |
| `npm run lint` | ESLint |
| `npm run deploy` | Build and publish to https://allenye.me |
| `./scripts/check-live.sh` | Poll until allenye.me serves the build in `dist/` |

## Adding stuff

- **A link:** add an entry to the `links` array in `src/App.tsx`.
- **A file (resume, image):** drop it in `public/`, then link to `/filename.pdf`.
- **Page title / description:** `index.html`.

## Infrastructure

| Piece | Where |
|---|---|
| Repo | https://github.com/allenye66/allenye66.github.io |
| Pages settings | https://github.com/allenye66/allenye66.github.io/settings/pages |
| Deploy history | https://github.com/allenye66/allenye66.github.io/deployments |
| DNS | Namecheap → allenye.me → Advanced DNS |
| Domain verification | https://github.com/settings/pages |

DNS records (don't delete):

| Type | Host | Value |
|---|---|---|
| A | `@` | `185.199.108.153`, `.109.153`, `.110.153`, `.111.153` |
| CNAME | `www` | `allenye66.github.io.` |
| TXT | `_github-pages-challenge-allenye66` | GitHub verification code |

`public/CNAME` must contain `allenye.me`; if it's missing, a deploy detaches the custom domain.

## Troubleshooting

- **Deployed but still see the old site.** Wait a minute, then hard-refresh. `./scripts/check-live.sh` tells you when it's actually live.
- **Site 404s on allenye.me.** Check `public/CNAME` exists, and that Pages → Custom domain still says `allenye.me`.
- **HTTPS error.** In Pages settings, make sure "Enforce HTTPS" is on; the cert is issued automatically.
- **`gh-pages` push fails with auth error.** Run `gh auth status`; re-login with `gh auth login` if needed.
