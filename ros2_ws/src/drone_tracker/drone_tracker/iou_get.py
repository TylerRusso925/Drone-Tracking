# detection_utils.py

import numpy as np

def calculate_iou(box1, box2):
    """
    Calculate the Intersection over Union (IoU) of two bounding boxes.

    Parameters:
    box1 (list or array): Bounding box [x1, y1, x2, y2]
    box2 (list or array): Bounding box [x1, y1, x2, y2]

    Returns:
    float: IoU value
    """
    # Calculate the (x, y)-coordinates of the intersection rectangle
    xA = max(box1[0], box2[0])
    yA = max(box1[1], box2[1])
    xB = min(box1[2], box2[2])
    yB = min(box1[3], box2[3])

    # Compute the area of intersection rectangle
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

    # Compute the area of both the prediction and ground-truth rectangles
    box1Area = (box1[2] - box1[0] + 1) * (box1[3] - box1[1] + 1)
    box2Area = (box2[2] - box2[0] + 1) * (box2[3] - box2[1] + 1)

    # Compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the intersection area
    iou = interArea / float(box1Area + box2Area - interArea)

    return iou

def get_max_iou_detection(detections, prev_box):
    """
    Get the detection with the maximum IoU with respect to the previous box.

    Parameters:
    detections (list): List of detections, each represented as [x1, y1, x2, y2]
    prev_box (list): Previous bounding box [x1, y1, x2, y2]

    Returns:
    list: Detection with the highest IoU
    """
    return max(detections, key=lambda det: calculate_iou(prev_box, [det[0], det[1], det[2], det[3]]))

