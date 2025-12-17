"""
Train a gesture recognizer model using MediaPipe Model Maker.
Saves the trained model as a .task bundle for use with MediaPipe.
Requires a prepared dataset.
Set enviromnment variables to customize paths:
  GESTURE_DATASET_DIR : path to dataset root
                        (default: ./gesture_dataset)
  GESTURE_EXPORT_DIR  : path to export trained model
                        (default: ./exported_model)
"""

import os
from mediapipe_model_maker import gesture_recognizer


def assert_dataset_ok(dataset_dir: str) -> None:
    """
    Check if the dataset directory is valid.
    Args:
        dataset_dir: Path to the dataset directory.
    Raises:
        FileNotFoundError: If the dataset directory does not exist.
        ValueError: If no label subfolders are found or if
                    'none' label folder is missing.
    """
    if not os.path.isdir(dataset_dir):
        raise FileNotFoundError(f"Dataset dir not found: {dataset_dir}")

    labels = sorted(
        [
            d
            for d in os.listdir(dataset_dir)
            if os.path.isdir(os.path.join(dataset_dir, d))
        ]
    )
    if not labels:
        raise ValueError(f"No label subfolders found in: {dataset_dir}")

    if not any(lbl.lower() == "none" for lbl in labels):
        raise ValueError(
            "Dataset must include a 'none' label folder (case-insensitive)."
        )

    print(f"Found {len(labels)} labels:")
    print("  " + ", ".join(labels))


def main():
    """
    Main function to train the gesture recognizer model.
    Sets up dataset paths, trains the model, evaluates it,
    and exports the trained model.
    1) Load dataset (runs hand detection + extracts landmarks;
       images w/ no hands are omitted)
    2) Split: 80% train, 10% val, 10% test
    3) Train
    4) Evaluate
    5) Export .task bundle for MediaPipe
    """
    dataset_dir = os.environ.get("GESTURE_DATASET_DIR", "./gesture_dataset")
    export_dir = os.environ.get("GESTURE_EXPORT_DIR", "./exported_model")

    assert_dataset_ok(dataset_dir)

    # 1) Load dataset (runs hand detection + extracts landmarks;
    #    images w/ no hands are omitted)
    # You can tune min_detection_confidence if too many samples
    # are being dropped.
    data = gesture_recognizer.Dataset.from_folder(
        dirname=dataset_dir,
        hparams=gesture_recognizer.HandDataPreprocessingParams(
            # shuffle=True is default in the notebook, but leaving
            # default is fine.
            # min_detection_confidence=0.5,
        ),
    )

    # 2) Split: 80% train, 10% val, 10% test
    train_data, rest_data = data.split(0.8)
    validation_data, test_data = rest_data.split(0.5)

    # 3) Train
    hparams = gesture_recognizer.HParams(
        export_dir=export_dir,
        epochs=20,
        batch_size=32,
        learning_rate=1e-3,
        # steps_per_epoch=1000,
    )

    model_options = gesture_recognizer.ModelOptions(
        dropout_rate=0.1,
        # layer_widths=[128, 64],  # optional: add MLP layers
    )

    options = gesture_recognizer.GestureRecognizerOptions(
        hparams=hparams,
        model_options=model_options,
    )

    model = gesture_recognizer.GestureRecognizer.create(
        train_data=train_data,
        validation_data=validation_data,
        options=options,
    )

    # 4) Evaluate
    loss, acc = model.evaluate(test_data, batch_size=32)
    print(f"Test loss: {loss:.4f}  Test accuracy: {acc:.4f}")

    # 5) Export .task bundle for MediaPipe
    # saves to hparams.export_dir as gesture_recognizer.task
    model.export_model()
    out_task = os.path.join(export_dir, "gesture_recognizer.task")
    print(f"Exported: {out_task}")


if __name__ == "__main__":
    main()
