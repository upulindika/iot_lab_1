import io
import re

import picamera
from PIL import Image
from tflite_runtime.interpreter import Interpreter

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class HouserBoon:
    args = {
        "labels": "/home/pi/picar-4wd/coco_labels.txt",
        "model": "/home/pi/picar-4wd/detect.tflite",
        "threshold": .4
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


import numpy as np
import math
import time
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.servo import Servo
from picar_4wd.speed import Speed
import picar_4wd as fc
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class LightningMcqueen:
    ser = Servo(PWM("P0"))
    us = Ultrasonic(Pin("D8"), Pin("D9"))
    angle_increment = 2
    length_per_position = 5.5  # 5 cm / position in numpy array
    houser_boon = HouserBoon()
    target = (150, 150)
    direction = 0

    def __init__(self, target):
        self.target = target
        self.map_to_fill = np.ones((150, 150))
        self.position = (self.map_to_fill.shape[0] * .5, 0)
        self.rescan_limit = 10

    def findMove(self, current, prev):
        if current[1] == prev[1] - 1:
            return "up"
        if current[1] == prev[1] + 1:
            return "down"
        if current[0] == prev[0] + 1:
            return "forward"

    def lets_do_this_thing(self, map_to_fill):
        map_to_fill = self.scan_and_build_map(60, map_to_fill)
        np.set_printoptions(threshold=np.inf)

        grid = Grid(matrix=map_to_fill)
        start = grid.node(0, int(map_to_fill.shape[0] * .5))
        end = grid.node(self.target[0], self.target[1])
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        print('operations:', runs, 'path length:', len(path))
        print(grid.grid_str(path=path, start=start, end=end))
        print(path)
        direction = self.direction
        prev = self.position


        # direction 0 = east
        # direction 1 = south
        # direction -1 = north

        for coordinate in path:
            print("Moving to this location", coordinate)
            print("Moving to this direction", direction)
            move = self.findMove(coordinate, prev)
            while (self.houser_boon.show_us_the_way() == "person"):
                time.sleep(3)
                print("STOP at a Person")
            if (self.houser_boon.show_us_the_way() == "stop sign"):
                time.sleep(5)
                print("STOP at a stop sign")

            if direction == 0 and move == "forward":
                print("Forward direction == 0")
                self.move25()
                tempList = list(self.position)
                tempList[0] += self.length_per_position
                self.position = tuple(tempList)

            elif direction == 1 and move == "forward":
                print("Forward direction == 1")
                self.turnLeft()
                self.move25()
                direction = 0
                tempList = list(self.position)
                tempList[0] += self.length_per_position
                self.position = tuple(tempList)

            elif direction == -1 and move == "forward":
                print("Forward direction == -1")
                self.turnRight()
                self.move25()
                direction = 0
                tempList = list(self.position)
                tempList[0] += self.length_per_position
                self.position = tuple(tempList)

            if direction == 0 and move == "down":
                print("Down direction == 0")
                self.turnRight()
                self.move25()
                self.move25()
                direction = 1
                tempList = list(self.position)
                tempList[1] -= self.length_per_position
                self.position = tuple(tempList)

            elif direction == 1 and move == "down":
                print("Down direction == 1")
                self.move25()
                tempList = list(self.position)
                tempList[1] -= self.length_per_position
                self.position = tuple(tempList)

            elif direction == -1 and move == "down":
                print("Down direction == -1")
                self.turnRight()
                self.turnRight()
                self.move25()
                self.move25()
                direction = 1
                tempList = list(self.position)
                tempList[1] -= self.length_per_position
                self.position = tuple(tempList)

            elif direction == 0 and move == "up":
                print("Up direction == 0")
                self.turnLeft()
                self.move25()
                self.move25()
                direction = -1
                tempList = list(self.position)
                tempList[1] += self.length_per_position
                self.position = tuple(tempList)

            elif direction == 1 and move == "up":
                print("Up direction == 1")
                self.turnLeft()
                self.turnLeft()
                self.move25()
                self.move25()
                direction = -1
                tempList = list(self.position)
                tempList[1] += self.length_per_position
                self.position = tuple(tempList)

            elif direction == -1 and move == "up":
                print("Up direction == -1")
                self.move25()
                tempList = list(self.position)
                tempList[1] += self.length_per_position
                self.position = tuple(tempList)

            prev = coordinate

        tempTarget = list(self.target)
        tempTarget[0] -= self.position[0]
        tempTarget[1] -= self.position[1]
        self.target = tuple(tempTarget)

        self.direction = direction

    def kachow(self):
        while (self.lets_do_this_thing(self.map_to_fill)):
            print("Scanned")

    def move25(self):
        speed4 = Speed(25)
        speed4.start()
        # time.sleep(2)
        fc.forward(100)
        x = 0
        for i in range(1):
            # time.sleep(0.1)
            speed = speed4()
            x += speed * 0.1
            print("%smm/s" % speed)
        print("%smm" % x)
        speed4.deinit()
        fc.stop()

    def turnLeft(self):
        speed4 = Speed(25)
        speed4.start()
        # time.sleep(2)
        fc.turn_left(80)
        x = 0
        for i in range(6):
            time.sleep(0.1)
            speed = speed4()
            x += speed * 0.1
            print("%smm/s" % speed)
        print("%smm" % x)
        speed4.deinit()
        fc.stop()

    def turnRight(self):
        speed4 = Speed(25)
        speed4.start()
        # time.sleep(2)
        fc.turn_right(80)
        x = 0
        for i in range(6):
            time.sleep(0.1)
            speed = speed4()
            x += speed * 0.1
            print("%smm/s" % speed)
        print("%smm" % x)
        speed4.deinit()
        fc.stop()

    def get_distance(self, angle):
        self.ser.set_angle(angle)
        time.sleep(0.5)
        return self.us.get_distance()

    def scan_and_build_map(self, angle, map_to_fill):
        last_position = [0, 0]
        for current_angle in range(-1 * angle, angle, self.angle_increment):
            current_distance = self.get_distance(current_angle)
            print(current_distance)
            if current_angle == -1 * angle:
                last_position = [(len(map_to_fill) * .5 - current_distance * math.sin(
                    math.radians(current_angle))) / self.length_per_position,
                                 current_distance * math.cos(math.radians(current_angle)) / self.length_per_position]
                if (last_position[0] < len(map_to_fill) and last_position[1] <= len(map_to_fill)):
                    map_to_fill[int(last_position[0]), int(last_position[1])] = 0
            else:
                current_position = [len(map_to_fill) * .5 - current_distance * math.sin(
                    math.radians(current_angle)) / self.length_per_position,
                                    current_distance * math.cos(math.radians(current_angle)) / self.length_per_position]
                if (current_position[0] < len(map_to_fill) and current_position[1] <= len(map_to_fill)):
                    map_to_fill[int(current_position[0]), int(current_position[1])] = 0
                    slope = (current_position[1] - last_position[0]) / (current_position[0] - last_position[0])
                    for i in range(0, int(current_position[0] - last_position[0])):
                        if (last_position[0] + i < len(map_to_fill) and last_position[1] + i * slope < len(
                                map_to_fill)):
                            map_to_fill[int(last_position[0] + i), int(last_position[1] + i * slope)] = 0
        return map_to_fill


SPEED = LightningMcqueen((50, 75))

SPEED.kachow()
