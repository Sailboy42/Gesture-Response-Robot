from __future__ import annotations

import argparse
import os
import shutil
from collections import Counter
from pathlib import Path

try:
    import yaml  # pip install pyyaml
except ImportError as e:
    raise SystemExit("Missing dependency: pyyaml. Install with: pip install pyyaml") from e


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

def norm_label(s: str) -> str:
    s = s.strip().lower()
    out = []
    for ch in s:
        out.append(ch if ch.isalnum() else "_")
    s = "".join(out)
    while "__" in s:
        s = s.replace("__", "_")
    return s.strip("_")

def load_names(data_yaml: Path) -> list[str]:
    data = yaml.safe_load(data_yaml.read_text())
    names = data.get("names")
    if isinstance(names, dict):
        names = [names[i] for i in sorted(names.keys())]
    if not isinstance(names, list) or not names:
        raise ValueError("Could not find a valid 'names:' list in data.yaml")
    return [norm_label(n) for n in names]

def read_yolo_label_file(lbl_path: Path) -> list[int]:
    if not lbl_path.exists():
        return []
    text = lbl_path.read_text().strip()
    if not text:
        return []
    class_ids = []
    for line in text.splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        try:
            class_ids.append(int(float(parts[0])))
        except ValueError:
            continue
    return class_ids

def pick_class(class_ids: list[int], policy: str) -> int | None:
    if not class_ids:
        return None
    uniq = set(class_ids)
    if policy == "first":
        return class_ids[0]
    if policy == "majority":
        return Counter(class_ids).most_common(1)[0][0]
    if policy == "skip_multi":
        return class_ids[0] if len(uniq) == 1 else None
    raise ValueError(f"Unknown policy: {policy}")

def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)

def copy_or_link(src: Path, dst: Path, mode: str) -> None:
    ensure_dir(dst.parent)
    if dst.exists():
        return
    if mode == "copy":
        shutil.copy2(src, dst)
    elif mode == "hardlink":
        os.link(src, dst)
    elif mode == "symlink":
        dst.symlink_to(src.resolve())
    else:
        raise ValueError(f"Unknown mode: {mode}")

def convert_one_split(images_dir: Path, labels_dir: Path, out_split_dir: Path,
                      names: list[str], policy: str, mode: str) -> dict[str, int]:
    counts = Counter()

    # Create folders
    for lbl in ["none"] + names:
        ensure_dir(out_split_dir / lbl)

    # Walk images
    for img in images_dir.iterdir():
        if not img.is_file():
            continue
        if img.suffix.lower() not in IMG_EXTS:
            continue

        lbl_file = labels_dir / (img.stem + ".txt")
        class_ids = read_yolo_label_file(lbl_file)
        chosen = pick_class(class_ids, policy)

        if chosen is None:
            # No boxes => none. Multi-class skipped if policy=skip_multi.
            if not class_ids:
                label_name = "none"
            else:
                counts["skipped"] += 1
                continue
        else:
            if 0 <= chosen < len(names):
                label_name = names[chosen]
            else:
                counts["skipped"] += 1
                continue

        dst = out_split_dir / label_name / img.name
        copy_or_link(img, dst, mode)
        counts[label_name] += 1

    return dict(counts)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project_root", required=True,
                    help="Folder containing data.yaml and dataset_split/")
    ap.add_argument("--out_root", default="mp_dataset",
                    help="Output folder (created under project_root unless absolute)")
    ap.add_argument("--policy", choices=["first", "majority", "skip_multi"], default="skip_multi",
                    help="How to label images with multiple boxes/classes")
    ap.add_argument("--mode", choices=["copy", "hardlink", "symlink"], default="copy",
                    help="How to place images in output (copy is safest)")
    args = ap.parse_args()

    project_root = Path(args.project_root).resolve()
    data_yaml = project_root / "data.yaml"
    split_root = project_root / "dataset_split"

    if not data_yaml.exists():
        raise FileNotFoundError(f"Missing {data_yaml}")
    if not split_root.exists():
        raise FileNotFoundError(f"Missing {split_root}")

    names = load_names(data_yaml)
    print("[info] class names:", names)

    out_root = Path(args.out_root)
    out_root = (project_root / out_root) if not out_root.is_absolute() else out_root
    ensure_dir(out_root)

    summary = {}
    for split in ["train", "val", "test"]:
        images_dir = split_root / "images" / split
        labels_dir = split_root / "labels" / split
        if not images_dir.exists():
            print(f"[warn] Missing {images_dir}, skipping {split}")
            continue
        if not labels_dir.exists():
            print(f"[warn] Missing {labels_dir}, skipping {split}")
            continue

        out_split_dir = out_root / split
        ensure_dir(out_split_dir)
        counts = convert_one_split(images_dir, labels_dir, out_split_dir, names, args.policy, args.mode)
        summary[split] = counts
        print(f"[done] {split}: {counts}")

    print("\n[summary]")
    for split, counts in summary.items():
        print(f"{split}: {counts}")
    print("\nOutput at:", out_root)

if __name__ == "__main__":
    main()
