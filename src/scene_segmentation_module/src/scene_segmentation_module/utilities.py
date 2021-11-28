# These functions are taken from the OpenVino Jupyter Notebooks
# https://github.com/openvinotoolkit/openvino_notebooks

import numpy as np
import cv2
from typing import List, NamedTuple, Optional, Tuple


class Label(NamedTuple):
    index: int
    color: Tuple
    name: Optional[str] = None

class SegmentationMap(NamedTuple):
    labels: List

    def get_colormap(self):
        return np.array([label.color for label in self.labels])

    def get_labels(self):
        labelnames = [label.name for label in self.labels]
        if any(labelnames):
            return labelnames
        else:
            return None

cityscape_labels = [
    Label(index=0, color=(128, 64, 128), name="road"),
    Label(index=1, color=(244, 35, 232), name="sidewalk"),
    Label(index=2, color=(70, 70, 70), name="building"),
    Label(index=3, color=(102, 102, 156), name="wall"),
    Label(index=4, color=(190, 153, 153), name="fence"),
    Label(index=5, color=(153, 153, 153), name="pole"),
    Label(index=6, color=(250, 170, 30), name="traffic light"),
    Label(index=7, color=(220, 220, 0), name="traffic sign"),
    Label(index=8, color=(107, 142, 35), name="vegetation"),
    Label(index=9, color=(152, 251, 152), name="terrain"),
    Label(index=10, color=(70, 130, 180), name="sky"),
    Label(index=11, color=(220, 20, 60), name="person"),
    Label(index=12, color=(255, 0, 0), name="rider"),
    Label(index=13, color=(0, 0, 142), name="car"),
    Label(index=14, color=(0, 0, 70), name="truck"),
    Label(index=15, color=(0, 60, 100), name="bus"),
    Label(index=16, color=(0, 80, 100), name="train"),
    Label(index=17, color=(0, 0, 230), name="motorcycle"),
    Label(index=18, color=(119, 11, 32), name="bicycle"),
    Label(index=19, color=(255, 255, 255), name="background"),
]

CityScapesSegmentation = SegmentationMap(cityscape_labels)

binary_labels = [
    Label(index=0, color=(255, 255, 255), name="background"),
    Label(index=1, color=(0, 0, 0), name="foreground"),
]

BinarySegmentation = SegmentationMap(binary_labels)

def segmentation_map_to_image(
    result: np.ndarray, colormap: np.ndarray, remove_holes=False
) -> np.ndarray:
    """
    Convert network result of floating point numbers to an RGB image with
    integer values from 0-255 by applying a colormap.

    :param result: A single network result after converting to pixel values in H,W or 1,H,W shape.
    :param colormap: A numpy array of shape (num_classes, 3) with an RGB value per class.
    :param remove_holes: If True, remove holes in the segmentation result.
    :return: An RGB image where each pixel is an int8 value according to colormap.
    """
    if len(result.shape) != 2 and result.shape[0] != 1:
        raise ValueError(
            f"Expected result with shape (H,W) or (1,H,W), got result with shape {result.shape}"
        )

    if len(np.unique(result)) > colormap.shape[0]:
        raise ValueError(
            f"Expected max {colormap[0]} classes in result, got {len(np.unique(result))} "
            "different output values. Please make sure to convert the network output to "
            "pixel values before calling this function."
        )
    elif result.shape[0] == 1:
        result = result.squeeze(0)

    result = result.astype(np.uint8)

    contour_mode = cv2.RETR_EXTERNAL if remove_holes else cv2.RETR_TREE
    mask = np.zeros((result.shape[0], result.shape[1], 3), dtype=np.uint8)
    for label_index, color in enumerate(colormap):
        label_index_map = result == label_index
        label_index_map = label_index_map.astype(np.uint8) * 255
        contours, hierarchies = cv2.findContours(
            label_index_map, contour_mode, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(
            mask,
            contours,
            contourIdx=-1,
            color=color.tolist(),
            thickness=cv2.FILLED,
        )

    return mask


def segmentation_map_to_overlay(
    image, result, alpha, colormap, remove_holes=False
) -> np.ndarray:
    """
    Returns a new image where a segmentation mask (created with colormap) is overlayed on
    the source image.

    :param image: Source image.
    :param result: A single network result after converting to pixel values in H,W or 1,H,W shape.
    :param alpha: Alpha transparency value for the overlay image.
    :param colormap: A numpy array of shape (num_classes, 3) with an RGB value per class.
    :param remove_holes: If True, remove holes in the segmentation result.
    :return: An RGP image with segmentation mask overlayed on the source image.
    """
    if len(image.shape) == 2:
        image = np.repeat(np.expand_dims(image, -1), 3, 2)

    mask = segmentation_map_to_image(result, colormap, remove_holes)
    return cv2.addWeighted(mask, alpha, image, 1 - alpha, 0)


