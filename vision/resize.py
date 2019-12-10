from PIL import Image
import torch
import torch.nn.functional as F
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import numpy as np

def resize(image, size):
    image = F.interpolate(image.unsqueeze(0), size=size, mode="nearest").squeeze(0)
    return image

def pad_to_square(img, pad_value):
    c, h, w = img.shape
    dim_diff = np.abs(h - w)
    # (upper / left) padding and (lower / right) padding
    pad1, pad2 = dim_diff // 2, dim_diff - dim_diff // 2
    # Determine padding
    pad = (0, 0, pad1, pad2) if h <= w else (pad1, pad2, 0, 0)
    # Add padding
    img = F.pad(img, pad, "constant", value=pad_value)

    return img, pad

def getitem():
    img_path = "image_8.png"
    # Extract image as PyTorch tensor
    img = transforms.ToTensor()(Image.open(img_path))
    # Pad to square resolution
    img, _ = pad_to_square(img, 0)
    # Resize
    img = resize(img, 640)

    return img

img = getitem()
img = transforms.functional.to_pil_image(img)
plt.imshow(img)
plt.show()