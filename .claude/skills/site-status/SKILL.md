---
name: site-status
description: Diagnose allenye.me when it looks broken, is down, shows an old version, or has an HTTPS or domain error. Checks DNS, GitHub Pages config, HTTPS, and what's actually being served.
---

# Check allenye.me health

Run these and compare against the expected values.

```sh
dig +short A allenye.me                 # expect 185.199.108.153 / 109 / 110 / 111
dig +short CNAME www.allenye.me         # expect allenye66.github.io.
dig +short TXT _github-pages-challenge-allenye66.allenye.me   # expect a quoted code
gh api repos/allenye66/allenye66.github.io/pages \
  --jq '{status, cname, https_enforced, source, cert: .https_certificate.state}'
# expect: status built, cname allenye.me, https_enforced true,
#         source gh-pages "/", cert approved
cat public/CNAME                        # expect allenye.me
curl -sI https://allenye.me | head -1   # expect HTTP/2 200
./scripts/check-live.sh                 # is the local dist/ build the one being served?
```

## Common fixes

- **cname is null / site 404s:** `public/CNAME` was missing on the last deploy.
  Restore it, then redeploy. Or set it directly:
  `gh api -X PUT repos/allenye66/allenye66.github.io/pages -f cname=allenye.me`
- **"domain already taken" when setting cname:** the TXT verification record was
  removed. Re-add it in Namecheap (value is shown at https://github.com/settings/pages
  under allenye.me) and re-verify there.
- **DNS records wrong:** fix them in Namecheap → Domain List → allenye.me →
  Advanced DNS. Changing DNS needs the user's Namecheap session in Chrome. Confirm
  with the user before deleting any record.
- **HTTPS not enforced:** once the cert is `approved`:
  `gh api -X PUT repos/allenye66/allenye66.github.io/pages -F https_enforced=true`
- **status errored:** look at the latest run at
  https://github.com/allenye66/allenye66.github.io/deployments, then redeploy.

Report what you checked, what was wrong, and what you changed.
