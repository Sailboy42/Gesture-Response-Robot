# Gesture-Response-Robot

[![Docs deploy status](https://github.com/Sailboy42/Gesture-Response-Robot/actions/workflows/deploy-docs.yml/badge.svg?branch=main)](https://github.com/Sailboy42/Gesture-Response-Robot/actions/workflows/deploy-docs.yml)

[Visit the live site (when deployed): https://Sailboy42.github.io/Gesture-Response-Robot](https://Sailboy42.github.io/Gesture-Response-Robot)

Use a neato that can read a person’s hand gestures to do certain functions e.g
stop hand to make the neato stop, or circling your finger for it to spin or a
snapshot motion for the neato to take a photo.

## Website

This repository contains a Vite + React website for the Gesture Response Robot
project. The site source is in the `docs/` folder.

## Repository layout

- `docs/` — Vite + React website source (run the dev server with
  `cd docs && npm ci && npm run dev`).
- `docs/dist/` — Built static output after `npm run build`. Do not track
  built artifacts in git; they are ignored by `.gitignore`.
- `gesture_custom/` — Python scripts for gesture recognition model training
  and augmentation.
- `Gesture_neato/` — ROS package for robot control and gesture response.

## Deployment

This project uses GitHub Actions workflows to build and deploy the site:

**Docs Deployment**: The `deploy-docs.yml` workflow builds the `docs/` site and
publishes the production output to the `gh-pages` branch. It runs on push to
`main`, or you can trigger it manually from the Actions tab.

**Python Linting**: The `python-lint.yml` workflow checks Python files for PEP8
style violations using flake8.

What the deploy workflow does:

- Installs Node (v18), runs `npm ci` in `docs/` and runs `npm run build`.
- Publishes the generated `docs/dist/` to the `gh-pages` branch using
  `peaceiris/actions-gh-pages` and the repository's `GITHUB_TOKEN`.

How to verify and use:

- Push to `main` and open the repository Actions tab to watch the build & deploy
  run.
- After the run completes, the `gh-pages` branch will contain the built static
  site. In GitHub → Settings → Pages, set the source to the `gh-pages` branch
  (if not already set) to serve the site at
  `https://<your-org-or-username>.github.io/<repo-name>`.

Local build (quick check)

```bash
cd docs
npm ci
npm run build
# built files appear in docs/dist/
```

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

## Dataset Samples

A small `dataset_samples/` folder is included for visualization only.
It contains a few randomly selected images per class and is **not**
used for training.

The full dataset is excluded due to size and licensing considerations.

### Dataset Sources

The gesture recognition model was trained on data from the following sources:

- **Roboflow Offensive Gesture Dataset**: [https://universe.roboflow.com/comprobo/offensive-gesture-phknd](https://universe.roboflow.com/comprobo/offensive-gesture-phknd)
- **HaGRID 30K Sample (Kaggle)**: [https://www.kaggle.com/datasets/innominate817/hagrid-sample-30k-384p](https://www.kaggle.com/datasets/innominate817/hagrid-sample-30k-384p)
