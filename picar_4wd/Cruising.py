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


# This class controls the car navigation (speed / steering) accordingly
class Cruising:
    # Initializing the servo
    ser = Servo(PWM("P0"))
    # Initializing ultrasonic
    us = Ultrasonic(Pin("D8"), Pin("D9"))
    angle_increment = 5
    length_per_position = 5.5  # 11 cm / position in numpy array
    detectObjects = DetectObjects()
    target = (150, 150)

    def __init__(self, target):
        self.target = target
        self.generated_map = np.ones((150, 150))
        self.position = (0, self.generated_map.shape[0] * .5)

    #find the next possible move based on current and previous coordinate
    def find_move(self, current, prev):
        if current[1] == prev[1] - 1:
            return "up"
        if current[1] == prev[1] + 1:
            return "down"
        if current[0] == prev[0] + 1:
            return "forward"

    def lets_cruise(self):
        generated_map = self.scan_and_build_map(90)
        print(generated_map)
        grid = Grid(matrix=generated_map)
        # start node is same coordinate that of self.position
        start = grid.node(0, int(generated_map.shape[0] * .5))
        end = grid.node(self.target[0], self.target[1])
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        print('operations:', runs, 'path length:', len(path))
        print(grid.grid_str(path=path, start=start, end=end))
        print(path)

        prev = self.position

        # Iterate optimal path and navigate the course
        for coordinate in path:
            print("Moving to this location", coordinate)
            move = self.find_move(coordinate, prev)

            # The car would stop moving if it detects a person to avoid a collision.
            while self.detectObjects.show_us_the_way() == "person":
                time.sleep(3)
                print("STOP at a Person")

            # The car must come to a complete stop and wait 5 seconds before it continues past the sign.
            if self.detectObjects.show_us_the_way() == "stop sign":
                time.sleep(5)
                print("STOP at a stop sign")
            # The car does a forward movement based on current and previous coordinate
            if move == "forward":
                print("Forward direction ")
                self.move()
                temp_list = list(self.position)
                temp_list[0] += self.length_per_position
                self.position = tuple(temp_list)
            # The car does a down movement based on current and previous coordinate
            elif move == "down":
                print("Down direction ")
                self.turnRight()
                self.move()
                self.turnLeft()
                temp_list = list(self.position)
                temp_list[1] -= self.length_per_position
                self.position = tuple(temp_list)
            # The car does a up movement based on current and previous coordinate
            elif move == "up":
                print("Up direction ")
                self.turnLeft()
                self.move()
                self.turnRight()
                temp_list = list(self.position)
                temp_list[1] += self.length_per_position
                self.position = tuple(temp_list)

            prev = coordinate

        temp_target = list(self.target)
        temp_target[0] -= self.position[0]
        temp_target[1] -= self.position[1]
        self.target = tuple(temp_target)

    # Move forward
    def move(self):
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

    # Turn left
    def turnLeft(self):
        speed4 = Speed(25)
        speed4.start()
        # time.sleep(2)
        fc.turn_left(60)
        x = 0
        for i in range(6):
            time.sleep(0.1)
            speed = speed4()
            x += speed * 0.1
            print("%smm/s" % speed)
        print("%smm" % x)
        speed4.deinit()
        fc.stop()

    # Turn right
    def turnRight(self):
        speed4 = Speed(25)
        speed4.start()
        # time.sleep(2)
        fc.turn_right(60)
        x = 0
        for i in range(6):
            time.sleep(0.1)
            speed = speed4()
            x += speed * 0.1
            print("%smm/s" % speed)
        print("%smm" % x)
        speed4.deinit()
        fc.stop()

    # could have used fc.get_distance_a() but need to delay the servo for better accuracy
    def get_distance(self, angle):
        self.ser.set_angle(angle)
        time.sleep(0.5)
        return self.us.get_distance()

    # referred from https://github.com/mccaesar/iot-labs/blob/2e36e9d8fd50ece2d999aad7231dd24ddc46d98a/iot-lab-1/navigation.py
    def scan_and_build_map(self, angle):
        map_to_fill = self.generated_map
        last_position = [0, 0]
        for current_angle in range(-1 * angle, angle, self.angle_increment):
            # could have used fc.get_distance_a() but need to deplay the servo for better accuracy
            current_distance = self.get_distance(current_angle)
            print(current_distance)
            if current_angle == -1 * angle:
                last_position = [(len(map_to_fill) * .5 - current_distance * math.sin(
                    math.radians(current_angle))) / self.length_per_position,
                                 current_distance * math.cos(math.radians(current_angle)) / self.length_per_position]
                if last_position[0] < len(map_to_fill) and last_position[1] <= len(map_to_fill):
                    map_to_fill[int(last_position[0]), int(last_position[1])] = 0
            else:
                current_position = [len(map_to_fill) * .5 - current_distance * math.sin(
                    math.radians(current_angle)) / self.length_per_position,
                                    current_distance * math.cos(math.radians(current_angle)) / self.length_per_position]
                if current_position[0] < len(map_to_fill) and current_position[1] <= len(map_to_fill):
                    map_to_fill[int(current_position[0]), int(current_position[1])] = 0
                    slope = (current_position[1] - last_position[0]) / (current_position[0] - last_position[0])
                    for i in range(0, int(current_position[0] - last_position[0])):
                        if (last_position[0] + i < len(map_to_fill) and last_position[1] + i * slope < len(
                                map_to_fill)):
                            map_to_fill[int(last_position[0] + i), int(last_position[1] + i * slope)] = 0
        return map_to_fill

def main():
    car = Cruising(target=(70, 70))
    car.lets_cruise()


if __name__ == '__main__':
    try:
        main()
    finally:
        fc.stop()