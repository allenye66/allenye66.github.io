---
name: deploy
description: Deploy the site to https://allenye.me. Use when the user says deploy, ship, publish, push it live, or update the site. Builds, commits and pushes source to main, publishes dist/ to the gh-pages branch, and verifies the new build is live.
---

# Deploy allenye.me

Run from the repo root.

1. **Check the tree.** `git status --short`. If there are uncommitted changes,
   show the user a one-line summary of what changed.

2. **Build.** `npm run build`. If it fails (type or build errors), fix the error
   if it's obvious and in code you or the user just wrote. Otherwise stop and
   show the error. Never deploy a failed build.

3. **Guard the domain.** `cat public/CNAME` must print `allenye.me`. If it's
   missing or different, restore it (`printf allenye.me > public/CNAME`) and
   tell the user, because deploying without it detaches the custom domain.

4. **Commit and push source.** If there are changes:
   `git add -A && git commit -m "<short summary of the change>"` then `git push`.
   Write the message from the actual diff. Skip if the tree is clean.

5. **Publish.** `npm run deploy`. Expect `Published` at the end.

6. **Verify.** `./scripts/check-live.sh` (takes up to ~5 min, usually <90s).
   It polls https://allenye.me until it serves the JS bundle hash in `dist/`.

7. **Report** in one or two lines: what changed, the commit, and that
   https://allenye.me is live. If verification timed out, say so plainly and
   point to https://github.com/allenye66/allenye66.github.io/deployments.

## Notes

- `npm run deploy` publishes the working tree, not `main`. That's why step 4
  commits first, so `main` always matches what's live.
- Rolling back: `git revert <sha>` (or `git checkout <sha> -- <file>`), then
  deploy again. There's no separate rollback button.
- Never hand-edit or force-push the `gh-pages` branch; every deploy rewrites it.
