import picar_4wd as fc
import time
import numpy as np

# map is 15x15 with divisions of 10cm
# (0, 0) - (14, 14)
# car's initial position before traversing the map is (7, 0)
#
# 14    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  <
# 13    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
# 12    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
# 11    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
# 10    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  9    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  8    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  7    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0   140cm
#  6    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  5    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  4    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  3    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  2    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  1    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
#  0    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0 <
#  y    ^       70cm         ^       70cm         ^
#     x 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14

division = 10 #in cm
index = 15

min_angle = -90
max_angle = 90
step_angle = 1

x_max_ind = (index-1)
y_max_ind = (index-1)
x_max_dist = division*x_max_ind
y_max_dist = division*y_max_ind
map_max_dist = np.sqrt((x_max_dist**2+y_max_dist**2))
car_x_offset = (index-1)/2

def main():
    global division, index
    #Initialize map as all zeros, should make it easier to place obsticles
    my_map = np.zeros((index,index))
    #Step 1 map obsticles
    map_obsticles(my_map)
    #Step 2 create a route, add offset for size of car

    #Step 3 OpenCV print what the camera sees

    #Step 4 move to target on map

    #Repeat

def map_obsticles(map_object):
    global division, index, x_max_dist, y_max_dist, map_max_dist, min_angle, max_angle, step_angle
    
    print(map_object)
    print()
    print()
    print()
    #print(division, " - ", index, " - ", x_max_dist, " - ", y_max_dist, " - ", map_max_dist)
    #sin(angle) = opposite/hypotenuse
    #cos(angle) = adjacent/hypotenuse
    

    for cur_angle in range(min_angle, max_angle+1, step_angle):
        obsticle_dist = fc.get_distance_at(cur_angle)
        #obsticle_dist = 70 #for debugging withou the car
        #translate cur_angle to angle perpendicular to face of car
        #to calculate coordinate
        if cur_angle < 0:
            temp_angle = cur_angle+90
            center_offset = -1
        else:
            temp_angle = 90-cur_angle
            center_offset = 1

        if temp_angle == 0:
            #Special case
            y = 0
            x = int(car_x_offset + ( center_offset * round(obsticle_dist/10) ))
        elif temp_angle == 90:
            #special case
            x = 7
            y = int(round(obsticle_dist/10))
        else:
            #calculate x,y coordinates
            y = np.sin(np.radians(temp_angle)) * obsticle_dist
            x = np.sqrt((obsticle_dist**2 - y**2))
            y = int(round(y/10))
            x = int(car_x_offset + (center_offset * round(x/10)))


        if x < 0 or x > x_max_ind or y > y_max_ind:
            #outside of range for the map so don't use it
            print(temp_angle, " - ", "Outside of map range ", x, ",", y)
        elif y < 0:
            #something went wrong, should never get a negative
            print(temp_angle, " - ", "Y index value error - ", y)
        else:
            #add 1 to index of map
            #print(temp_angle, " - ", x,",",y)
            map_object[x,y] = 1

    print(map_object)

if __name__ == "__main__":
    try:
        while True:
            #loop for ever, or till something crashes
            main()
            time.sleep(10)

    finally:
        print("End")
        #fc.stop()
