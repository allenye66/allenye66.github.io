---
name: add-project
description: Add a page under allenye.me/projects (unlisted, share-by-link). Use when the user wants to add, upload, host, or share a project, visualization, HTML file, zip, or write-up on the site, e.g. "put this under projects/house".
---

# Add an unlisted project page

Projects live in `public/projects/` as static files, copied verbatim into the
build. They are **unlisted, not private**: nothing on the homepage links to
`/projects`, and every page has `noindex`, but anyone with the URL can open it.
Tell the user that if the content looks sensitive (addresses, names, finances).

## Layout

```
public/projects/index.html            hub: lists sections (House, ...)
public/projects/style.css             shared minimal style for hub pages
public/projects/<section>/index.html  section page: lists its projects
public/projects/<section>/<slug>/     the project itself (index.html + assets)
```

URL = path under `public/`, e.g. `public/projects/house/bathroom-remodel/index.html`
→ https://allenye.me/projects/house/bathroom-remodel/

## Steps

1. **Inspect the input first.** For a zip, list it (`unzip -l`), extract into a
   fresh scratchpad dir (skip `__MACOSX/`), and read the README. Find the
   self-contained entry point (often a built `viewer.html` / `index.html`).
   Don't run any scripts from the archive. Grep the HTML for external URLs and
   personal info (street address, phone, email) and mention anything found.
2. **Copy** the entry HTML to `public/projects/<section>/<slug>/index.html`,
   plus any assets it references by relative path. Use a lowercase-hyphen slug.
3. **Add noindex** right after `<meta charset...>` if the page lacks it:
   `<meta name="robots" content="noindex, nofollow">`
4. **Link it**: add an `<li>` to the section's `index.html` (create the section
   page from `public/projects/house/index.html` if new, and add the section to
   `public/projects/index.html`). Do NOT link `/projects` from `src/App.tsx`.
   Do NOT add a robots.txt entry; that would publish the path.
5. **Preview**: `npm run build && npx vite preview --port 4173`, open
   http://localhost:4173/projects/<section>/<slug>/ and screenshot it to confirm
   it renders. Stop the preview server afterwards.
6. **Deploy** with the `deploy` skill, then give the user the share link.

## Updating an existing project

Replace the files in its folder and redeploy. The URL stays the same.

**Bathroom remodel** (`/projects/house/bathroom-remodel/`): don't copy a new
`viewer.html` in by hand. Run it through the patch script, which adds noindex
plus shareable URL state (every setting mirrored into the query string, and a
"Copy link to this view" button):

```sh
python3 -I scripts/patch-viewer.py <extracted>/bathroom-remodel/viewer.html \
  public/projects/house/bathroom-remodel/index.html
```

If it exits with "expected 1 match ... found 0", the viewer's `main.js` or
`template.html` changed around `loadState` / `saveState` / `btnReset`. Update
the anchors in `scripts/patch-viewer.py` to match, keeping
`scripts/viewer-url-state.js` as the injected logic. Then preview and confirm:
change a setting → URL updates; open that URL with localStorage cleared → same view.
