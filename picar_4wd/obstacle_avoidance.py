import picar_4wd as fc
import time
import random

speed = 10

def main():
    random_number = random.randint(0, 1)
    while True:
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
