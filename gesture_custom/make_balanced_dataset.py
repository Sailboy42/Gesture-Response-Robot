"""
Create a balanced dataset from an imbalanced gesture image dataset.
Creates symbolic links (or copies if symlinks not supported)
to a new output directory.
Adjusts the number of samples per class to achieve balance,
with special handling for the "none" class.
Requires the source dataset to be organized in
subfolders per class label.
"""

import os
import random
import shutil
from pathlib import Path
from collections import Counter

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def list_images(label_dir: Path):
    """
    Return list of image file paths in the given label directory.
    Args:
        label_dir: Path to the label directory.
    Returns:
        List of image file paths.
    """
    out = []
    for p in label_dir.rglob("*"):
        if p.suffix.lower() in IMG_EXTS and (p.is_file() or p.is_symlink()):
            out.append(p)
    return out


def safe_link(src: Path, dst: Path):
    """
    Create a symbolic link from src to dst.
    If symlinks are not supported, copy the file instead.
    Args:
        src: Source file path.
        dst: Destination file path.
    Raises:
        OSError: If linking or copying fails.
    """
    dst.parent.mkdir(parents=True, exist_ok=True)
    src_abs = src.resolve()  # absolute path

    try:
        os.symlink(str(src_abs), str(dst))
    except OSError:
        shutil.copy2(src_abs, dst)


def main(
    src_root: str = "./gesture_dataset",
    out_root: str = "./dataset_balanced",
    seed: int = 42,
    max_per_non_none: int = 6000,
    none_cap_ratio_vs_median: float = 1.5,
    iloveyou_min_target: int = 1700,
):
    """
    Main function to create a balanced dataset.
    Args:
        src_root: Path to the source dataset root directory.
        out_root: Path to the output balanced dataset root
                  directory.
        seed: Random seed for reproducibility.
        max_per_non_none: Maximum number of samples per
                          non-"none" class.
        none_cap_ratio_vs_median: Cap for "none" class samples
                                  as a ratio of the median
                                  non-"none" class count.
        iloveyou_min_target: Minimum target samples for the
                             "iloveyou" class.
    Raises:
        ValueError: If the source dataset is not organized in
                    the expected format.
    """
    random.seed(seed)
    src_root = Path(src_root)
    out_root = Path(out_root)

    labels = sorted([d.name for d in src_root.iterdir() if d.is_dir()])
    if not any(label.lower() == "none" for label in labels):
        raise ValueError("Missing required 'none' label folder.")

    # Load file lists
    files_by_label = {}
    for label in labels:
        files = list_images(src_root / label)
        if not files:
            print(f"WARNING: label '{label}' has 0 images.")
        files_by_label[label] = files

    counts = {k: len(v) for k, v in files_by_label.items()}
    print("Raw counts:", counts)

    # Compute target counts
    non_none_counts = [
        counts[label]
        for label in labels
        if label.lower() != "none" and counts[label] > 0
    ]
    median_non_none = (
        sorted(non_none_counts)[len(non_none_counts) // 2] if non_none_counts else 0
    )
    none_label = next(label for label in labels if label.lower() == "none")

    targets = {}
    for label in labels:
        if label.lower() == "none":
            targets[label] = int(
                min(
                    counts[label],
                    max(1, round(median_non_none * none_cap_ratio_vs_median)),
                )
            )
        elif label.lower() == "iloveyou":
            targets[label] = int(
                max(
                    min(counts[label], max_per_non_none),
                    min(iloveyou_min_target, max_per_non_none),
                )
            )
        else:
            targets[label] = int(min(counts[label], max_per_non_none))

    print("Target counts:", targets)

    # Create output (fresh)
    if out_root.exists():
        shutil.rmtree(out_root)
    out_root.mkdir(parents=True, exist_ok=True)

    # Sample + (optionally) repeat for upsampling
    for label in labels:
        src_files = files_by_label[label]
        tgt = targets[label]
        if not src_files or tgt <= 0:
            continue

        chosen = []
        if tgt <= len(src_files):
            chosen = random.sample(src_files, tgt)
        else:
            # upsample by repetition
            chosen = list(src_files)
            while len(chosen) < tgt:
                chosen.append(random.choice(src_files))

        # Write links/copies
        for i, src in enumerate(chosen):
            # keep extension for sanity
            dst = out_root / label / f"{label}_{i:07d}{src.suffix.lower()}"
            safe_link(src, dst)

    # Print final counts
    final = {}
    for label in labels:
        final[label] = len(list_images(out_root / label))
    print("Balanced counts:", final)
    print(f"Done. Output dataset: {out_root.resolve()}")


if __name__ == "__main__":
    main()
