#!/usr/bin/env python3

"""
Find an image whose YOLO-format label file contains a given
class id and open it.

Usage examples:
  python open_image_by_class.py --dataset
  ./offensive\\ gesture.v1i.darknet/test --class-id 1

This will search recursively for image files and check the
corresponding .txt label files in the same directory.
If a label file contains the specified class id
(as the first token on any line),
the image path will be printed and the image will be opened
using PIL.Image.show().

Used for inspecting datasets.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List


def find_images_with_class(dataset_dir: Path, class_id: int) -> List[Path]:
    """
    Return list of image paths whose corresponding .txt label file
    contains class_id.

    Assumes YOLO label files with the same basename and a .txt
    extension where each line has format:
    <class> <x> <y> <w> <h> (or similar).
    Matches if the first token == class_id.
    """
    image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
    matches: List[Path] = []

    for path in dataset_dir.rglob("*"):
        if path.suffix.lower() in image_exts:
            label_file = path.with_suffix(".txt")
            if label_file.exists():
                try:
                    text = label_file.read_text(encoding="utf-8")
                except Exception:
                    # fallback to latin-1 if encoding weird
                    try:
                        text = label_file.read_text(encoding="latin-1")
                    except Exception:
                        continue

                for line in text.splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    is_match = (
                        parts and parts[0].isdigit() and int(
                            parts[0]) == class_id
                    )
                    if is_match:
                        matches.append(path)
                        break
    return matches


def open_image(path: Path) -> None:
    """
    Open the image at the given path using the default image viewer.
    Args:
        path: Path to the image file.
    Raises:
        Exception: If the image cannot be opened.
    """
    try:
        from PIL import Image
    except Exception as e:
        print(
            "Pillow is required to open images " "(pip install pillow).",
            file=sys.stderr,
        )
        raise

    img = Image.open(path)
    img.show()


def main(argv: List[str] | None = None) -> int:
    """
    Main entry point for the script.
    Args:
        argv: List of command-line arguments. If None, uses sys.argv.
    Returns:
        Exit code: 0 on success, non-zero on failure.
    """
    parser = argparse.ArgumentParser(
        description="Open an image by YOLO class id")
    parser.add_argument(
        "--dataset",
        "-d",
        type=Path,
        required=True,
        help="Path to dataset root (will be searched recursively)",
    )
    parser.add_argument(
        "--class-id",
        "-c",
        type=int,
        default=1,
        help="Class id to match in label files (default: 1)",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the first matching image using the default image viewer",
    )
    parser.add_argument(
        "--list-all",
        action="store_true",
        help="List all matching image paths instead of stopping at the first",
    )

    args = parser.parse_args(argv)

    dataset_dir = args.dataset
    if not dataset_dir.exists():
        print(f"Dataset path does not exist: {dataset_dir}", file=sys.stderr)
        return 2

    matches = find_images_with_class(dataset_dir, args.class_id)
    if not matches:
        print(
            f"No images found with class id {
                args.class_id} in label files under {dataset_dir}"
        )
        return 1

    if args.list_all:
        for p in matches:
            print(p)
    else:
        chosen = matches[0]
        print(chosen)
        if args.open:
            try:
                open_image(chosen)
            except Exception as e:
                print(f"Failed to open image: {e}", file=sys.stderr)
                return 3

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
