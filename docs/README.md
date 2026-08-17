# docs

- `viewer/` — interactive URDF viewer (GitHub Pages: Settings → Pages → Deploy from branch `main`, folder `/docs`).
  Loads `../../s1_description/…` when served from the repo root, else falls back to raw.githubusercontent.com.
  `?arm=left`, `?q=j0,…,j6` (initial pose), `?sweep` (animate).
- `VERIFY_ON_HARDWARE.md` — checks to run on a real S1A/S1L before release.
- `AGENT_PROMPT_lerobot_plugin.md` — how the plugin was scoped.
