"""
Prepare a gesture recognition dataset from the
'offensive gesture.v1i.darknet' dataset.
Filters for 'middle finger' as positive examples and
'L' gesture as negative examples.
Excludes 'rock' gesture images entirely.
Saves the prepared dataset in a structure compatible with
MediaPipe Model Maker.
Requires the original dataset to be downloaded and extracted.
Uses:
- offensive gesture.v1i.darknet/train/
- offensive gesture.v1i.darknet/valid/
- offensive gesture.v1i.darknet/test/
"""

import shutil
from pathlib import Path

# Root where 'dataset' folder lives
ROOT = Path(".")
DATASET_DIR = ROOT / "offensive gesture.v1i.darknet"
OUTPUT_DIR = ROOT / "gesture_dataset"

MIDDLE_FINGER_CLASS = 0
ROCK_CLASS = 1
L_CLASS = 2

MIDDLE_FINGER_NAME = "middle_finger"
NONE_NAME = "none"

mf_dir = OUTPUT_DIR / MIDDLE_FINGER_NAME
none_dir = OUTPUT_DIR / NONE_NAME
mf_dir.mkdir(parents=True, exist_ok=True)
none_dir.mkdir(parents=True, exist_ok=True)

splits = ["train", "valid", "test"]

for split in splits:
    split_dir = DATASET_DIR / split
    if not split_dir.is_dir():
        print(f"Skipping missing split: {split_dir}")
        continue

    print(f"Processing {split_dir}")

    for img_path in split_dir.iterdir():
        if img_path.suffix.lower() not in [".jpg", ".jpeg", ".png"]:
            continue

        label_path = split_dir / (img_path.stem + ".txt")
        if not label_path.exists():
            # no labels — you can choose to ignore these or send them to none
            continue

        with open(label_path, "r") as f:
            lines = [line.strip() for line in f if line.strip()]

        if not lines:
            continue

        # Collect all class IDs present in this image
        class_ids = set()
        for line in lines:
            parts = line.split()
            try:
                cid = int(parts[0])
                class_ids.add(cid)
            except BaseException:
                continue

        # --- Filtering rules ---
        # 1) If image contains rock symbol → EXCLUDE ENTIRELY
        if ROCK_CLASS in class_ids:
            print(f"Skipping ROCK gesture image: {img_path}")
            continue

        # 2) If image contains middle finger → positive example
        if MIDDLE_FINGER_CLASS in class_ids:
            dest_dir = mf_dir
        else:
            # 3) Images containing L gesture → negative examples
            # (these go into none/)
            if L_CLASS in class_ids:
                dest_dir = none_dir
            else:
                # optional: skip all other images
                # if you want to keep them as negatives, uncomment:
                # dest_dir = none_dir
                continue

        dest_path = dest_dir / img_path.name
        shutil.copy(img_path, dest_path)
        print(f"{img_path} -> {dest_path}")

print("DONE!")
print(f"Middle finger images: {mf_dir}")
print(f"None (L-gesture only): {none_dir}")
