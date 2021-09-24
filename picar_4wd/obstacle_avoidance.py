import picar_4wd as fc
import time
import random

# Constant speed of 10
speed = 10

def main():
    # Random number to decide on random direction of left or right
    random_number = random.randint(0, 1)
    while True:
        # Ultrasonic scan
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        print(tmp)
        if tmp != [2, 2, 2, 2]:
            fc.stop()
            time.sleep(.5)
            fc.backward(5)
            time.sleep(.5)

            # if random_number is 0, car would go right
            if random_number == 0:
                fc.turn_right(speed)
            else:
                fc.turn_left(speed)
        else:
            fc.forward(speed)
            random_number = random.randint(0, 1)


if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
