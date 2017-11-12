import numpy as np
import tensorflow as tf

import utils.tools as tools
from object_detection.utils import label_map_util


class ObjectDetection:
    def __init__(self,
                 config,
                 im_width,
                 im_height,
                 gpu_usage=0.8,
                 threshold=0.5):
        self.category_index = {}
        self.config = {}
        self.tf_session = None
        self.detection_graph = None
        self.im_width = im_width
        self.im_height = im_height
        self.config = config
        self.gpu_usage = gpu_usage
        self.threshold = threshold

    def start_up(self):
        print("[OBJECT DETECTION] starting {} Graph".format(
            self.config['model_name']))
        label_map = label_map_util.load_labelmap(self.config["label_map"])
        categories = label_map_util.convert_label_map_to_categories(
            label_map,
            max_num_classes=self.config["num_classes"],
            use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.config["path_to_ckpt"], 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

                config = tf.ConfigProto()
                config.gpu_options.per_process_gpu_memory_fraction = self.gpu_usage

            self.tf_session = tf.Session(
                config=config, graph=self.detection_graph)
        print("[OBJECT DETECTION] started")

        return self

    def shutdown(self):
        self.tf_session.close()
        print("[OBJECT DETECTION] stopped")

    def detect_objects(self, image):
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)

        image_tensor = self.detection_graph.get_tensor_by_name(
            'image_tensor:0')
        boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name(
            'detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name(
            'num_detections:0')

        # Actual detection.
        (boxes, scores, classes, num_detections) = self.tf_session.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        return self.finalize_result(
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32), np.squeeze(scores))

    def finalize_result(self, boxes, classes, scores, max_results=20):

        if boxes.shape[0] < max_results:
            max_results = boxes.shape[0]

        detections = []

        for i in range(min(max_results, boxes.shape[0])):
            if scores is None or scores[i] > self.threshold:
                if classes[i] in self.category_index.keys():
                    class_name = self.category_index[classes[i]]['name']
                    class_id = int(classes[i])
                else:
                    class_id = -1
                    class_name = 'N/A'

                if scores is None:
                    score = -1
                else:
                    score = int(100 * scores[i])

                ymin, xmin, ymax, xmax = tuple(boxes[i].tolist())
                box = dict(
                    minY=int(ymin * self.im_height),
                    minX=int(xmin * self.im_width),
                    maxY=int(ymax * self.im_height),
                    maxX=int(xmax * self.im_width))
                detections.append({
                    "class_id": class_id,
                    "class_name": class_name,
                    "roi": box,
                    "score": score
                })

        result = dict(
            timeStamp=tools.get_timestamp_ms(), recognizedObjects=detections)

        return result
