import numpy as np
import cv2
import colorsys

def find_centers_and_colors(boxes: np.array) -> tuple:
    """
    Find the centers and colors of the given boxes.
    
    :param boxes: An array of bounding boxes.
    :return: A tuple containing the centers, colors, and verified boxes.
    """
    centers = []
    colors = []
    boxes_verified = []
    u_1 = []
    u_2 = []
    v_1 = []
    v_2 = []

    for box in boxes:
        if box[-2] < -1:
            continue
        center = [int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)]
        centers.append(center)
        colors.append(int(box[-1]))
        u_1.append(int(box[0]))
        u_2.append(int(box[2]))
        v_1.append(int(box[1]))
        v_2.append(int(box[3]))
        boxes_verified.append(box)
    return centers, colors, boxes_verified, u_1, u_2, v_1, v_2

def get_box_color(color_code: int) -> tuple:
    """
    Get the box color based on the given color code.
    
    :param color_code: An integer representing the color code.
    :return: A tuple representing the box color in BGR format.
    """
    if color_code == 0:
        return (0, 255, 0)
    elif color_code == 1:
        return (0, 0, 255)
    elif color_code == 2:
        return (0, 255, 255)

def draw_boxes(img, boxes, colors):
    """
    Draw the bounding boxes on the given image.
    
    :param img: The input image.
    :param boxes: A list of bounding boxes.
    :param colors: A list of colors corresponding to the boxes.
    :return: The image with the drawn bounding boxes.
    """
    for i, box in enumerate(boxes):
        box = np.rint(box).astype(int)
        box_color = get_box_color(colors[i])
        img = cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), box_color, 2)

    return img

def parse_result(img, boxes, draw=True):
    """
    Parse the results and draw the bounding boxes on the image if required.
    
    :param img: The input image.
    :param boxes: An array of bounding boxes.
    :param draw: A boolean indicating whether to draw the boxes on the image.
    :return: The image, centers, and colors of the bounding boxes.
    """
    centers, colors, boxes_verified, u_1, u_2, v_1, v_2 = find_centers_and_colors(boxes)

    if draw:
        img = draw_boxes(img, boxes_verified, colors)

    return img, centers, colors, u_1, u_2, v_1, v_2
