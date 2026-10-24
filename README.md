# allenye.me

Personal site. Vite + React + TypeScript, hosted on GitHub Pages at https://allenye.me.

## Develop

```sh
npm install
npm run dev
```

## Deploy

```sh
npm run deploy
```

Builds to `dist/` and pushes it to the `gh-pages` branch, which GitHub Pages serves.
`main` holds source only, so pushing to `main` does not change the live site.
The custom domain comes from `public/CNAME`.
