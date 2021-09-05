import picar_4wd as fc
import time
import random

speed = 20

my_angle_max = 90
my_angle_min = -90

driving_angle_max = 90
driving_angle_min = -90

my_angle_step = 10

obsticle_list = []

def main():
    #Making sure we use the global values
    global speed, driving_angle_max, driving_angle_min, obsticle_list

    #Scan ultrasonic sensor and build a list of obsticles
    for x in range(driving_angle_min, driving_angle_max, my_angle_step):
        #be conscious of how we build this list
        #reading distance left to right will tell us first index is to the left, and last index is to the right
        obsticle_list.append(fc.get_status_at(x))

    #reset servo to the left most reading
    fc.servo.set_angle(driving_angle_min)

    #hard coding the list split is just quicker and cleaner
    #Though we scan from min to max angle, we only car for what is in front of the car
    temp_list = obsticle_list[6:12]
    
    #list of all '2' indicates there is a clear path in front
    if temp_list == [2,2,2,2,2,2]:
        #no reason to stop, go forward!!!
        fc.forward(speed)
    else:
        #backup
        fc.stop()
        fc.backward(speed)
        time.sleep(2)
        fc.stop()
        #call the routine to avoid the obsticle
        avoid_obsticle()
    
    #reset the list, else it will keep growing
    obsticle_list = []

#end of main()


def avoid_obsticle():
    #Making sure we use the global values
    global speed, driving_angle_max, driving_angle_min, obsticle_list

    #reset the list, else it will keep growing
    obsticle_list = []
    turn_time = 2
    keep_checking = 1
    
    #pick a random direction as lab requests
    left_right = random.randint(0, 1)
    if left_right:
        local_speed = speed
    else:
        local_speed = -speed

    #turn for amount of time in seconds then check if clear
    fc.turn_right(local_speed)
    time.sleep(turn_time)
    fc.stop()

    while keep_checking:
        #mostly copy from above
        for x in range(driving_angle_min, driving_angle_max, my_angle_step):
            obsticle_list.append(fc.get_status_at(x))

        fc.servo.set_angle(driving_angle_min)
        temp_list = obsticle_list[6:12]
    
        if temp_list == [2,2,2,2,2,2]:
            #path is clear, stop turning
            keep_checking = 0
        else:
            #turn for amount of time in seconds then check if clear
            fc.turn_right(local_speed)
            time.sleep(turn_time)
            fc.stop()

#end of avoid_obsticle()


if __name__ == "__main__":
    try:
        while True:
            #loop for ever, or till something crashes
            main()
    finally: 
        fc.stop()
