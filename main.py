#!/usr/bin/env pybricks-micropython
import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color # type: ignore
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.iodevices import Ev3devSensor
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.messaging import BluetoothMailboxServer, TextMailbox, NumericMailbox
from collections import *
#from collections import counter

# Create your objects here.
ev3 = EV3Brick()

# Initialize the motors and sensors.
motor_left = Motor(Port.A) #Check for correct Port 
motor_right = Motor(Port.D) 
sensor_left= ColorSensor(Port.S1)
sensor_right= ColorSensor(Port.S4)
ultraSonic_front = UltrasonicSensor(Port.S2)
ultraSonic_back = UltrasonicSensor(Port.S3)

color_list_left = [(205,28,100),(211, 78, 65),(63,65,69),(120,16,77)] # 0+4=White, 1=Blue, 2=Yellow
color_list_right = [(205,28,100),(205,84,44),(86,70,55),(205,28,100)] # 0+4=White, 1=Blue, 2=Yellow

STATES=["DRIVE","STOP","SLOW","TURN_LEFT","TURN_RIGHT","SWITCH_LANE","HOLD", "PARK"] #All possible states the robot can have 
LANE_STATES=["UNKNOWN","LEFT_LANE","RIGHT_LANE"]
rounds=1
state=STATES[1]
lane_state=LANE_STATES[0]
Speed = -150
left_array=deque([0])
right_array=deque([0])
left = None
right = None
white_count = 0
isSEND=False

# set up server
server = BluetoothMailboxServer()
mbox =  TextMailbox('greeting',server)
server.wait_for_connection()
print('connected!')

# Initialize the drive base.
robot = DriveBase(motor_left, motor_right, wheel_diameter=55, axle_track=95) #Check for correct Parameter 

# calculating of rgb to hsv
# from https://tutorials.aposteriori.com.sg/110-Pybricks-Basics/99-Special-Topics/20-HSV-and-RGB.html
def rgb_to_hsv(rgb):
    hsv = [0, 0, 0]
    normRgb = [0, 0, 0]

    for i in range(3):
      normRgb[i] = rgb[i] / 100

    cMax = max(normRgb)
    cMin = min(normRgb)
    diff = cMax - cMin

    if cMax == cMin:
      hsv[0] = 0
    elif cMax == normRgb[0]:
      hsv[0] = 60 * (normRgb[1] - normRgb[2]) / diff
    elif cMax == normRgb[1]:
      hsv[0] = 60 * (2 + (normRgb[2] - normRgb[0]) / diff)
    else:
      hsv[0] = 60 * (4 + (normRgb[0] - normRgb[1]) / diff)

    if hsv[0] < 0:
      hsv[0] += 360

    if cMax == 0:
      hsv[1] = 0
    else:
      hsv[1] = diff / cMax * 100

    hsv[2] = cMax * 100

    return hsv

def proc(num, off):
    n = off*num
    return num-n, num+n

# calculates allowed range for hsv values
def procentRange(h,s,v, color_list):
    procH = proc(color_list[0],0.25) # 25
    procS = proc(color_list[1],0.30) # 30
    procV = proc(color_list[2],0.15) # 15
    if (procH[0]<= h <= procH[1]) and (procS[0]<= s <=procS[1]) and (procV[0] <= v <= procV[1]):
        return True
    return False

# finds color in list with allowed offset
def find_color(): #Update sensor readings
    global right, left
    h_l,s_l,v_l = rgb_to_hsv(sensor_left.rgb())
    h_r,s_r,v_r = rgb_to_hsv(sensor_right.rgb())
    if (h_l == 0 and s_l == 0 and v_l == 0) and (h_r == 0 and s_r == 0 and v_r == 0):
        return None, None
    for i in color_list_left:
        if procentRange(h_l,s_l,v_l,i):
            print("i_l=", i)
            left = i
            return left, None
        else:
            left = None

    for i in color_list_right:
        if procentRange(h_r,s_r,v_r,i):
            print("i_r =",i)
            right = i
            return None, right
        else:
            right = None

    if right == None and left == None:
        print('H: {0}\t S: {1}\t V: {2}'.format(h_l, s_l, v_l))
        print(rgb_to_hsv(sensor_right.rgb()))
        print("----------------\n")
    elif left == None:
        print('L -> H: {0}\t S: {1}\t V: {2}'.format(h_l, s_l, v_l))
    elif right == None:
        print(rgb_to_hsv(sensor_right.rgb()))
    
    # print("Left: ", left)
    # print("Right: ", right)
    return left,right

def update_front_back():
    if rounds <= 1:
        return ultraSonic_front.distance(), ultraSonic_back.distance()
    else: 
        return ultraSonic_front.distance(), 0
    
def update_sensors(): #Update sensor readings
    global left_array
    global right_array

    # color_left = find_color()
    # color_right = find_color()
    color_left = sensor_left.color()
    color_right = sensor_right.color()
    if (color_left == Color.BLUE or color_right == Color.BLUE) or (color_left == Color.YELLOW or color_right == Color.YELLOW): 
        color_left, color_right = find_color()
        
        left_array.append(color_num(color_left, color_list_left))
        right_array.append(color_num(color_right, color_list_right))
    else:
        left_array.append(color_num(color_left, color_list_left))
        right_array.append(color_num(color_right, color_list_right))
    

    if len(left_array)>1:
        left_array.popleft()
    if len(right_array)>1:
        right_array.popleft()
    color_left=most_common(left_array)
    color_right=most_common(right_array)

    # print(color_left)
    # print(right)
    return color_left, color_right

def most_common (array):
    count_dict={}

    for num in array:
        if num in count_dict:
         count_dict[num] += 1
        else:
            count_dict[num] = 1

    most_common_number = None
    max_count = 0

    for num, count in count_dict.items():
        if count > max_count:
            max_count = count
            most_common_number = num    
    return most_common_number
   
# function changes state depending of sensor outputs
def transition_state(color_left, color_right): 
    global state
    global lane_state
    global rounds
    global white_count
    global isSEND
    allowed_dist = 170
    front, back = update_front_back()

    # check for obstacle and change lane
    if front < allowed_dist:
        if isSEND==False:
            mbox.send("Obstacle")
            isSEND=True
        return STATES[5]
    if back < allowed_dist and rounds != 1: #parking lot
        print("Should Park")
        print(back)
        mbox.send("Parking")
        mbox.send("NONE")
        return STATES[7]

    if front > allowed_dist:
        #   switch lane and lanestate
        if left_color == 1 and right_color == 1: # Black
            return STATES[0] # Move forward

        if (left_color == 1 and right_color == 2 ): # Black / White
            if lane_state==LANE_STATES[0]:
                lane_state=LANE_STATES[1]
            return STATES[3]

        if (left_color == 2 and right_color == 1) or (left_color == 2): # White / Black
            if lane_state==LANE_STATES[0]:
                lane_state=LANE_STATES[2]
            return STATES[4]
        if left_color == 3: # Green
            if lane_state==LANE_STATES[0]:
                lane_state=LANE_STATES[1]
            white_count=0
            return STATES[4]

        if right_color == 3: # Green
            if lane_state==LANE_STATES[0]:
                lane_state=LANE_STATES[2]
            white_count=0
            return STATES[3]
        if left_color == 4 or right_color == 4: #Blue - 3 sec stop
            mbox.send("Hold")
            
            return STATES[6]

        if left_color == 5 or right_color == 5: #Yellow - slow down 
            mbox.send("Slow")
            
            return STATES[2]
    
        if left_color == 6 and right_color == 6: #Red - lane switch
            rounds=rounds-1

        if left_color==0 and right_color==0: # None
            return STATES[0]

def color_num(color, color_list):
    if color == Color.BLACK: # black
        return 1
    elif color == Color.WHITE or color == color_list[0] or color == color_list[3]: # white 
        return 2
    elif color == Color.GREEN: # green
        return 3
    elif color == color_list[1]:  # blue
        return 4
    elif color == color_list[2]: # yellow
        return 5
    elif color == Color.RED : # red
        return 6
    else: 
        return 0

#Executes operations depending of the state 
def switch(state):  
    global rounds
    global Speed
    global lane_state
    global white_count
    global isSEND
    front, back = update_front_back()
    if state ==  "DRIVE":
        if white_count !=0: # makes car turn at a angle depending on the amount of whites
            white_count=white_count-1
        robot.drive(Speed,0)
    elif state ==  "STOP":
        robot.drive(0,0)
    elif state ==  "SLOW":
        robot.drive(Speed/2,0)
        clear_array()
        wait(2000)
        mbox.send("NONE")

    elif state ==  "TURN_LEFT":
        robot.drive(Speed,30 + white_count) 
        white_count=50
        wait(50)
    elif state ==  "TURN_RIGHT":
        robot.drive(Speed,-30 - white_count)
        white_count=70
        wait(50)
    elif state ==  "SWITCH_LANE": 
        if lane_state=="LEFT_LANE":
            print("left")
            robot.drive(Speed,-70)
            wait(700)
            mbox.send("NONE")
            robot.drive(Speed,0)
            wait(700)
            robot.drive(Speed,90)
            wait(300)
            print("end")
            lane_state =LANE_STATES[2]
            
        elif lane_state=="RIGHT_LANE":
            print("right")
            robot.drive(Speed,70)
            wait(700)
            mbox.send("NONE")
            robot.drive(Speed,0)
            wait(500)
            robot.drive(Speed,-90)
            wait(300)
            print("end")
            lane_state = LANE_STATES[1]
            
        else:
            print("No lane detected")
            robot.drive(Speed,-45)
        mbox.send("NONE")
        isSEND=False
        clear_array()
    elif state == "HOLD":
        robot.drive(0,0)
        wait(3000)
        robot.drive(Speed,0)
        clear_array()
        wait(300)
        mbox.send("NONE")
    elif state == "PARK":
        robot.drive(Speed,0)
        wait(500)
        robot.turn(-100)
        while (front > 70):
            front, back = update_front_back()
            robot.drive(Speed, 0)
        robot.turn(100)
        ev3.speaker.beep(500,100)
        while (front > 50):
            front, back = update_front_back()
            robot.drive(Speed, 0)
        while(True):
            robot.drive(0,0)

# clear most common array
def clear_array():
    global left_array
    global right_array
    left_array=deque([])
    right_array=deque([])

def clear_lane():
    lane_state=LANE_STATES[0]

#main loop of the 2
while True:
    start=time.time()
    # Update sensor readings
    pressed = ev3.buttons.pressed()
    left_color, right_color = update_sensors()
    
    #connecting to client
   
    #mbox.wait()
   # print(mbox.read())
   # print(mbox.read())
    
    # Handle state transitions'
    switch(transition_state(left_color, right_color))
    # print(state)
    if pressed:
        print("reset")
        state=STATES[1]
        lane_state=LANE_STATES[0]
        rounds=1
        Speed=-150
        ev3.speaker.beep(500,100)
        pressed=False
    # print(time.time()-start)




