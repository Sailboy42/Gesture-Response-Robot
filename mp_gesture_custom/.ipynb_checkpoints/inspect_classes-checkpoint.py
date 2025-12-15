from pathlib import Path

DATASET_DIR = Path("offensive gesture.v1i.darknet")  # adjust if needed

class_ids = set()

for split in ["train", "valid", "test"]:
    split_dir = DATASET_DIR / split
    if not split_dir.is_dir():
        continue

    for label_path in split_dir.glob("*.txt"):
        with open(label_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue
                try:
                    cid = int(parts[0])
                    class_ids.add(cid)
                except ValueError:
                    continue

print("Found class IDs:", sorted(class_ids))
