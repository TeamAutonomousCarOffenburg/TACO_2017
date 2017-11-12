import argparse
import os
import cv2
from utils.visualisation import draw_boxes_and_labels


def create_parser():
    parser = argparse.ArgumentParser(description='Test visualisation')
    parser.add_argument('--image', required=True)
    return parser


if __name__ == '__main__':
    args = create_parser().parse_args()

    if not os.path.isfile(args.image):
        print("image does not exist")
        exit(1)

    results = [{
        "class_id": 10,
        "class_name": "crossroad",
        "roi": dict(minY=100, minX=100, maxX=200, maxY=200),
        "score": 70
    }, {
        "class_id": 20,
        "class_name": "person",
        "roi": dict(minY=100, minX=300, maxX=200, maxY=400),
        "score": 90
    }]

    image = cv2.imread(args.image)

    image = draw_boxes_and_labels(image, results)

    cv2.imshow("image", image)

    if cv2.waitKey(60000) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
