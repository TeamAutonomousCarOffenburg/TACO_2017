import numpy as np
import cv2
import utils.colors as c


def draw_boxes_and_labels(image, results, thickness=3):
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    font_scale = 0.5
    font_thickness = 1
    # Black (not in color dict)
    text_color = (0, 0, 0)

    for result in results:
        if result["class_id"] == -1:
            color = (255, 255, 255)
        else:
            # id = result["class_id"] % len(c.STANDARD_COLORS)
            color = list(c.STANDARD_COLORS.values())[result["class_id"]]

        box = result["roi"]
        image = cv2.rectangle(image, (box["minX"], box["minY"]),
                              (box["maxX"], box["maxY"]), color, thickness)

        text = "{}: {}%".format(result["class_name"], result["score"])

        textSize = cv2.getTextSize(text, font, font_scale, font_thickness)
        text_width, text_height = textSize[0]
        margin = int(np.ceil(0.05 * text_height))

        # we need faktor of 2 from the text_height for the right height for the text box
        image = cv2.rectangle(
            image, (box["minX"], box["minY"] - text_height * 2 - 2 * margin),
            (box["minX"] + text_width, box["minY"]), color, -1)

        text_position = (box["minX"], box["minY"] - text_height - margin)
        image = cv2.putText(image, text, text_position, font, font_scale,
                            text_color, font_thickness, cv2.LINE_AA)
    return image
