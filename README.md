# Gesture-Response-Robot
Use a neato that can read a person’s hand gestures to do certain functions e.g stop hand to make the neato stop, or circling your finger for it to spin or a snapshot motion for the neato to take a photo.

## Website

This repository contains a small static website used to host the project report and publish progress updates.

Files added
- `index.html` — Project report page and static Updates log
- `css/styles.css` — Readable report-oriented styles
- `js/main.js` — Minimal helper (mobile nav toggle)
- `assets/robot.svg` — Illustration (optional)

How to view locally
- Open `index.html` in your browser (double-click or use "Open With")
- Or serve the folder with a simple local server (recommended) from PowerShell:

```powershell
# from the repository root
python -m http.server 8000
# then open http://localhost:8000
```

Posting updates
- The site includes a static Updates section in `index.html` (search for the `#updates` section). To post an update, add a new `<article class="update">` entry with a `<time>` and a short note.

Updates workflow

This repository supports an updates-as-markdown workflow to keep the Updates log source-controlled and easy to author.

- Add updates as markdown files under `updates/`. Filenames should start with the date in `YYYY-MM-DD` form, e.g. `2025-11-17-initial-site.md`. The script also supports optional YAML front-matter fields: `title`, `author`, `date`, `summary`, and `image`.
- Use `scripts/new_update.py "Short title"` to scaffold a new update file with front-matter and today's date.
- `scripts/build_updates.py` converts markdown to HTML and injects the generated update entries into `index.html` between the markers `<!-- UPDATES-START -->` and `<!-- UPDATES-END -->`. It writes a ready-to-publish copy into `site/`.

Local build & preview

1. Install the Python dependencies into the interpreter you'll use for builds:

```powershell
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

2. Build the site locally and preview:

```powershell
python scripts\build_updates.py
# open site/index.html in a browser, or serve it:
python -m http.server 8000 --directory site
# then open http://localhost:8000
```

3. Run the smoke test (optional):

```powershell
python scripts\test_build.py
```

Authoring updates

- Create `updates/YYYY-MM-DD-your-title.md` or run the scaffold helper:

```powershell
python scripts\new_update.py "Add camera feed"
```

- Edit the generated markdown (add a short `summary:` and an optional `image:` path, e.g. `assets/example.png`) and commit the file.

CI and deployment

- The repository contains a GitHub Actions workflow at `.github/workflows/build-and-deploy.yml` that runs on pushes to `main` and `Website-Work`.
- Workflow steps:
	- Install Python dependencies.
	- Run `scripts/build_updates.py` and `scripts/test_build.py` (smoke test).
	- On successful test and only for pushes to `main`, the Action copies the generated `site/` into `docs/` and commits it to `main` (this makes the site immediately publishable via GitHub Pages configured to serve from `docs/`).

Deployment notes

- Current setup publishes the built site by copying `site/` into `docs/` on `main`. This is simple to review and does not require a separate `gh-pages` branch. If you prefer to keep generated artifacts out of `main`, we can change the workflow to publish to a `gh-pages` branch instead (I can update the Action for that).

Tips & troubleshooting

- If the build script fails with a missing Python package, ensure your `python` command refers to the same interpreter where `pip install -r requirements.txt` was run.
- Do not remove the markers `<!-- UPDATES-START -->` and `<!-- UPDATES-END -->` from `index.html`; they are required for the build script to inject updates.
- If you add local images referenced by `image:` front-matter, place them under `assets/` so they are copied into `site/assets/` during the build.