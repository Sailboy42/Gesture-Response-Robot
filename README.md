# Gesture-Response-Robot

Use a neato that can read a person’s hand gestures to do certain functions e.g
stop hand to make the neato stop, or circling your finger for it to spin or a
snapshot motion for the neato to take a photo.

## Website

This repository contains a minimal static site for the Gesture Response Robot
project.

Files: `index.html`, `css/styles.css`, `js/main.js`, `assets/`.

Quick preview:

```bash
python -m http.server 8000
```

## Repository layout

- `docs/` — Vite + React website source (run the dev server with
  `cd docs && npm ci && npm run dev`).
- `site/` or `docs/dist/` — (optional) built static output after
  `npm run build`. Do not track built artifacts in git; they are ignored by
  `.gitignore`.
- `hand_track.py`, `camera.py` (or `robot/` folder) — Python computer-vision
  code using OpenCV (run with Python 3.x; install dependencies e.g.
  `pip install opencv-python numpy`).

## Deployment

This project uses a GitHub Actions workflow to build the `docs/` site and
publish the production output to the `gh-pages` branch. The workflow will
run on push to `main` or `Prettier_Webpage`, or you can trigger it manually
from the Actions tab.

What the workflow does
- Installs Node (Node 18), runs `npm ci` in `docs/` and runs `npm run build`.
- Publishes the generated `docs/dist/` to the `gh-pages` branch using
  `peaceiris/actions-gh-pages` and the repository's `GITHUB_TOKEN`.

How to verify and use
- Push your branch (for example `Prettier_Webpage`) and open the repository
  Actions tab to watch the build & deploy run.
- After the run completes, the `gh-pages` branch will contain the built
  static site. In GitHub → Settings → Pages set the source to the `gh-pages`
  branch (if not already set) to serve the site at
  `https://<your-org-or-username>.github.io/<repo-name>`.

Local build (quick check)

```bash
cd docs
npm ci
npm run build
# built files appear in docs/dist/
```

If you'd like I can also add a short 'Deployment' badge or a small note in
the README that shows the live Pages URL after the first successful deploy.

## Quick run

- Run the website (dev):

```bash
cd docs
npm ci
npm run dev
# open the Vite URL printed in your terminal
```

- Run the robot/vision code (example):

```bash
python3 hand_track.py
```
