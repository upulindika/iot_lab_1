import io
import re

import numpy as np
import picamera
from PIL import Image
from tflite_runtime.interpreter import Interpreter

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class DetectObjects:
    ### from https://github.com/tensorflow/examples/blob/master/lite/examples/object_detection/raspberry_pi/detect_picamera.py
    args = {
        "labels": "/home/pi/picar-4wd/coco_labels.txt",
        "model": "/home/pi/picar-4wd/detect.tflite",
        "threshold": .6
    }

    def load_labels(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            labels = {}
            for row_number, content in enumerate(lines):
                pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
                if len(pair) == 2 and pair[0].strip().isdigit():
                    labels[int(pair[0])] = pair[1].strip()
                else:
                    labels[row_number] = pair[0].strip()
        return labels

    def __init__(self):
        self.labels = self.load_labels(self.args["labels"])
        self.interpreter = Interpreter(self.args["model"])
        self.interpreter.allocate_tensors()
        _, self.input_height, self.input_width, _ = self.interpreter.get_input_details()[0]['shape']

    def set_input_tensor(self, interpreter, image):
        """Sets the input tensor."""
        tensor_index = interpreter.get_input_details()[0]['index']
        input_tensor = interpreter.tensor(tensor_index)()[0]
        input_tensor[:, :] = image

    def get_output_tensor(self, interpreter, index):
        """Returns the output tensor at the given index."""
        output_details = interpreter.get_output_details()[index]
        tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
        return tensor

    def detect_objects(self, interpreter, image, threshold):
        """Returns a list of detection results, each a dictionary of object info."""
        self.set_input_tensor(interpreter, image)
        self.interpreter.invoke()

        # Get all output details
        boxes = self.get_output_tensor(self.interpreter, 0)
        classes = self.get_output_tensor(self.interpreter, 1)
        scores = self.get_output_tensor(self.interpreter, 2)
        count = int(self.get_output_tensor(self.interpreter, 3))

        results = []
        for i in range(count):
            if scores[i] >= threshold:
                result = {
                    'bounding_box': boxes[i],
                    'class_id': classes[i],
                    'score': scores[i]
                }
                results.append(result)
        return results

    def show_us_the_way(self):
        with picamera.PiCamera(resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=10) as camera:
            camera.start_preview()
            try:
                stream = io.BytesIO()
                camera.capture(stream, format="jpeg")
                stream.seek(0)
                image = Image.open(stream).convert('RGB').resize(
                    (self.input_width, self.input_height), Image.ANTIALIAS)

                results = self.detect_objects(self.interpreter, image, self.args["threshold"])
                for obj in results:
                    # TO-DO Need to find why it is always detecting person
                    if self.labels[obj['class_id']] == "person" or self.labels[obj['class_id']] == "stop sign":
                        print("self.labels[obj['class_id']] ", self.labels[obj['class_id']])
                        return self.labels[obj['class_id']]

                stream.seek(0)
                stream.truncate()

            finally:
                camera.stop_preview()
        return "Clear"
