"""
Augment images in the "iloveyou" gesture class to reach a target number of images.
Applies random transformations such as rotation, scaling, brightness/contrast adjustment,
blur, noise, cutout, and optional horizontal flipping.
"""
import argparse
import random
from pathlib import Path

import cv2
import numpy as np

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

def list_images(d: Path):
    """
    List all image files in a directory.
    Args:
        d: Path to the directory.
    Returns:
        List of image file paths.
    """
    return [p for p in d.rglob("*") if p.is_file() and p.suffix.lower() in IMG_EXTS]

def imread_any(path: Path):
    """
    Read an image from the given path using OpenCV.
    Args:
        path: Path to the image file.
    Returns:
        The image as a NumPy array.
    Raises:
        ValueError: If the image cannot be read.
    """
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"Failed to read: {path}")
    return img

def clamp01(x):
    """
    Clamp a value to the range [0, 1].
    Args:
        x: Input value.
    Returns:
        Clamped value.
    """
    return max(0.0, min(1.0, x))

def random_affine(img, rng: random.Random):
    """
    Apply a random affine transformation (rotation, scaling, translation) to the image.
    Args:
        img: Input image as a NumPy array.
        rng: Random number generator.
    Returns:
        Transformed image as a NumPy array.
    """
    h, w = img.shape[:2]

    # gentle ranges
    angle = rng.uniform(-12, 12)              # degrees
    scale = rng.uniform(0.90, 1.10)
    tx = rng.uniform(-0.06, 0.06) * w         # pixels
    ty = rng.uniform(-0.06, 0.06) * h

    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, angle, scale)
    M[0, 2] += tx
    M[1, 2] += ty

    out = cv2.warpAffine(
        img, M, (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101
    )
    return out

def random_brightness_contrast(img, rng: random.Random):
    """
    Apply random brightness and contrast adjustments to the image.
    Args:
        img: Input image as a NumPy array.
        rng: Random number generator.
    Returns:
        Adjusted image as a NumPy array.
    """
    # brightness/contrast jitter
    alpha = rng.uniform(0.85, 1.20)  # contrast
    beta = rng.uniform(-20, 20)      # brightness
    out = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    return out

def random_blur_noise(img, rng: random.Random):
    """
    Apply random blur and noise to the image.
    Args:
        img: Input image as a NumPy array.
        rng: Random number generator.
    Returns:
        Augmented image as a NumPy array.
    """
    out = img

    # occasional mild blur
    if rng.random() < 0.20:
        k = rng.choice([3, 5])
        out = cv2.GaussianBlur(out, (k, k), 0)

    # occasional mild gaussian noise
    if rng.random() < 0.30:
        noise_sigma = rng.uniform(3, 10)
        noise = np.random.normal(0, noise_sigma, out.shape).astype(np.float32)
        out_f = out.astype(np.float32) + noise
        out = np.clip(out_f, 0, 255).astype(np.uint8)

    return out

def random_cutout(img, rng: random.Random):
    """
    Apply random cutout (occlusion) to the image.
    Args:
        img: Input image as a NumPy array.
        rng: Random number generator.
    Returns:
        Augmented image as a NumPy array.
    """
    # small occlusion patch (simulate partial occlusion)
    if rng.random() > 0.25:
        return img
    h, w = img.shape[:2]
    patch_w = int(rng.uniform(0.06, 0.14) * w)
    patch_h = int(rng.uniform(0.06, 0.14) * h)
    x0 = rng.randint(0, max(0, w - patch_w))
    y0 = rng.randint(0, max(0, h - patch_h))

    out = img.copy()
    # fill with mean-ish color
    mean_color = [int(x) for x in out.mean(axis=(0, 1))]
    out[y0:y0+patch_h, x0:x0+patch_w] = mean_color
    return out

def maybe_flip(img, rng: random.Random, allow_flip: bool):
    """
    Maybe flip the image horizontally.
    Args:
        img: Input image as a NumPy array.
        rng: Random number generator.
        allow_flip: Whether to allow flipping.
    Returns:
        Flipped image as a NumPy array, or original image if not flipped.
    """
    # NOTE: flipping can change handedness; OFF by default.
    if allow_flip and rng.random() < 0.35:
        return cv2.flip(img, 1)
    return img

def augment_one(img, rng: random.Random, allow_flip: bool):
    """
    Apply a series of random augmentations to the image.
    Args:
        img: Input image as a NumPy array.
        rng: Random number generator.
        allow_flip: Whether to allow horizontal flipping.
    Returns:
        Augmented image as a NumPy array.
    """
    out = img
    out = random_affine(out, rng)
    out = random_brightness_contrast(out, rng)
    out = random_blur_noise(out, rng)
    out = random_cutout(out, rng)
    out = maybe_flip(out, rng, allow_flip)
    return out

def save_jpg(path: Path, img):
    """
    Save an image as a JPEG file.
    Args:
        path: Output file path.
        img: Input image as a NumPy array.
    Raises:
        ValueError: If the image is not uint8 or if saving fails.
    """
    if img.dtype != np.uint8:
        raise ValueError("Image must be uint8")
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(path), img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    if not ok:
        raise ValueError(f"Failed to write: {path}")

def main():
    """
    Main function to run the augmentation process.
    Parses command-line arguments, loads images from the source directory,
    applies augmentations, and saves the results to the destination directory.
    1) copy originals first
    2) generate augmented until target
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("--src_iloveyou", required=True, help="gestue_dataset/iloveyou")
    ap.add_argument("--dst_iloveyou", required=True, help="dataset_balanced/iloveyou")
    ap.add_argument("--target", type=int, default=2500, help="Total images to end up with")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--allow_flip", action="store_true", help="Enable horizontal flips (off by default)")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    src = Path(args.src_iloveyou)
    dst = Path(args.dst_iloveyou)

    src_imgs = list_images(src)
    if len(src_imgs) == 0:
        raise ValueError(f"No images found in {src}")

    # start fresh
    if dst.exists():
        for p in dst.rglob("*"):
            if p.is_file():
                p.unlink()
    dst.mkdir(parents=True, exist_ok=True)

    # 1) copy originals first
    originals_to_copy = min(len(src_imgs), args.target)
    chosen_originals = rng.sample(src_imgs, originals_to_copy) if originals_to_copy < len(src_imgs) else list(src_imgs)

    count = 0
    for p in chosen_originals:
        img = imread_any(p)
        out_path = dst / f"iloveyou_{count:06d}.jpg"
        save_jpg(out_path, img)
        count += 1

    # 2) generate augmented until target
    while count < args.target:
        base_path = rng.choice(src_imgs)
        base_img = imread_any(base_path)
        aug = augment_one(base_img, rng, args.allow_flip)
        out_path = dst / f"iloveyou_{count:06d}.jpg"
        save_jpg(out_path, aug)
        count += 1

    print(f"Done. Wrote {count} images to: {dst.resolve()}")

if __name__ == "__main__":
    main()
