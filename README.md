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
