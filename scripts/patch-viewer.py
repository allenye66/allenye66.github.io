"""Prepares the bathroom-remodel viewer.html for allenye.me.

Adds a noindex tag and shareable URL state (scripts/viewer-url-state.js): every
setting is mirrored into the query string, and a "Copy link to this view" button
sits next to Reset. Each patch anchors on exact text from the viewer's main.js
and template.html; if a new viewer version changes that text, this fails loudly
instead of shipping a half-patched page.

Usage:
    python3 -I scripts/patch-viewer.py <viewer.html> public/projects/house/bathroom-remodel/index.html
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def replace_once(html, old, new, what):
    count = html.count(old)
    if count != 1:
        sys.exit(f"patch-viewer: expected 1 match for {what}, found {count}. The viewer changed; update this script.")
    return html.replace(old, new)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    html = src.read_text(encoding="utf-8")
    url_js = (HERE / "viewer-url-state.js").read_text(encoding="utf-8")

    html = replace_once(
        html,
        '<meta charset="utf-8">',
        '<meta charset="utf-8">\n<meta name="robots" content="noindex, nofollow">',
        "charset meta",
    )

    # URL params win over localStorage, so a shared link opens as copied.
    html = replace_once(
        html,
        "function loadState() {\n  try { return JSON.parse(localStorage.getItem(STATE_KEY) || 'null'); }",
        url_js
        + "function loadState() {\n  try { const fromUrl = queryToState(); if (fromUrl) return fromUrl; } catch (e) { /* bad link: fall back */ }\n"
        "  try { return JSON.parse(localStorage.getItem(STATE_KEY) || 'null'); }",
        "loadState()",
    )

    html = replace_once(
        html,
        "  try { localStorage.setItem(STATE_KEY, JSON.stringify(collectState())); } catch (e) { /* storage blocked: settings just won't persist */ }",
        "  const st = collectState();\n"
        "  try { localStorage.setItem(STATE_KEY, JSON.stringify(st)); } catch (e) { /* storage blocked: settings just won't persist */ }\n"
        "  try { syncUrl(st); } catch (e) { /* URL sync is best-effort */ }",
        "saveState() body",
    )

    # Reset also clears the URL, otherwise the reload would restore the link's state.
    html = replace_once(
        html,
        "$('btnReset').addEventListener('click', () => { localStorage.removeItem(STATE_KEY); location.reload(); });",
        "$('btnReset').addEventListener('click', () => { localStorage.removeItem(STATE_KEY); location.replace(location.pathname); });\n"
        "  $('btnCopyLink').addEventListener('click', (e) => copyViewLink(e.currentTarget));",
        "reset handler",
    )

    html = replace_once(
        html,
        '<button id="btnReset">Reset view + settings</button><span class="muted" id="savedNote">Settings are saved in this browser.</span>',
        '<button id="btnCopyLink">Copy link to this view</button><button id="btnReset">Reset view + settings</button>'
        '<span class="muted" id="savedNote">Settings are saved in this browser and in the page link.</span>',
        "reset button",
    )

    dst.write_text(html, encoding="utf-8")
    print(f"patch-viewer: wrote {dst} ({dst.stat().st_size / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
