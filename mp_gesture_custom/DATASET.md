# Dataset (Not Included)

The gesture dataset is not included in this repository due to size
and licensing considerations.

## Structure
gesture_dataset/
  dislike/
  fist/
  iloveyou/
  like/
  middle_finger/
  none/
  one/
  palm/
  rock/

## Stats
- Total images: ~50k
- Classes: 9
- Heavy class imbalance (`none` dominant)
- `iloveyou` originally had 25 samples

## Reproducibility
- `make_balanced_dataset.py` creates a balanced training view
- `augment_iloveyou.py` expands `iloveyou` 1730 samples
- Training script: `train_gesture_recognizer.py`
