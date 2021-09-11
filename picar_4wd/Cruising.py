import math
import time

import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

import picar_4wd as fc
from DetectObjects import DetectObjects
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
from picar_4wd.servo import Servo
from picar_4wd.speed import Speed
from picar_4wd.ultrasonic import Ultrasonic


class Cruising:
    ser = Servo(PWM("P0"))
    us = Ultrasonic(Pin("D8"), Pin("D9"))
    angle_increment = 2
    length_per_position = 5.5  # 5 cm / position in numpy array
    detectObjects = DetectObjects()
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
            while (self.detectObjects.show_us_the_way() == "person"):
                time.sleep(3)
                print("STOP at a Person")
            if (self.detectObjects.show_us_the_way() == "stop sign"):
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

            elif direction == 0 and move == "down":
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


def main():
    car = Cruising(target=(50, 75))
    while car.lets_do_this_thing(car.map_to_fill):
        print("Scanned")


if __name__ == '__main__':
    main()
