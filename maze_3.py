# order to tune
'''
------------ 5 min ------------
starting position -> alley
heart -> alley
heart -> ending

------------ 5 min ------------
tune arm position (marker_calib.py) -> edit main
    - check if 200, -70 can pickup
    - check if 200, -70 can track marker
find optimal marker threshold (marker_calib.py) -> edit main 
assume optimal wall threshold (marker_calib.py) -> edit main
    - i think shld be quite near to wall but slighter more than marker threshold
starting position -> navigate grey

------------ 5 min ------------
collect coral

------------ 5 min ------------
navigate blue
merge grey and blue
'''


# global thresholds to tune
wallDistThreshold = 22
markerThreshold = 22

# pid for marker alignment
speed_pid = PIDCtrl()
angular_pid = PIDCtrl()

# global flags
coral_count = 0
picked_up = False
endRun = False

def initialise():
    # custom pid for marker alignment
    speed_pid.set_ctrl_params(5, 0, 0)
    angular_pid.set_ctrl_params(150, 0, 0)

    # default rotation speed
    chassis_ctrl.set_rotate_speed(100)

    # enable vision detection for markers (not needed for line tracking)
    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    vision_ctrl.set_marker_detection_distance(3)
    
    # reset gimbal
    gimbal_ctrl.recenter()

    # enable IR distance sensor
    ir_distance_sensor_ctrl.enable_measure(1)

    # reset arm and gripper to initial positions (arm down, gripper open)
    if robotic_arm_ctrl.get_position()[1] < 0: # to prevent arm from jamming if never reset properly
        robotic_arm_ctrl.move(0, 0-robotic_arm_ctrl.get_position()[1], wait_for_complete=True)
    robotic_arm_ctrl.recenter(wait_for_complete=True)
    gripper_ctrl.open()

def set_led(R, G, B):
    led_ctrl.set_bottom_led(rm_define.armor_bottom_front, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_left, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_right, R, G, B, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_back, R, G, B, rm_define.effect_always_on)


# ------------------------ movement helper functions ------------------------

def track_marker():
    global distance_from_marker
    marker_id = 0

    # get marker information
    # if multiple markers, use get_nearest_marker()
    marker_info = vision_ctrl.get_marker_detection_info()
    print("Marker found" if len(marker_info) > 2 else "No marker")
    if len(marker_info) > 2:
        marker_id = marker_info[1]
        print(marker_id)

        # get distance and set PID errors
        distance_from_marker = ir_distance_sensor_ctrl.get_distance_info(1)
        speed_pid.set_error(distance_from_marker)
        x_offset = marker_info[2] - 0.5
        angular_pid.set_error(x_offset)

        if marker_info[1] == 47:
            t = 50
        else:
            t = markerThreshold

        # adjust position until marker is close enough
        while distance_from_marker > t:
            chassis_ctrl.move_with_speed(0.5, 0, angular_pid.get_output())
            time.sleep(0.005)

            marker_info = vision_ctrl.get_marker_detection_info()
            distance_from_marker = ir_distance_sensor_ctrl.get_distance_info(1)
            speed_pid.set_error(distance_from_marker)
            if len(marker_info) > 2:
                # have to frikin check another time in case image is cut off and it throws some out of index error
                x_offset = marker_info[2] - 0.5
                angular_pid.set_error(x_offset)

        print("Final distance from marker", distance_from_marker)
        chassis_ctrl.stop()
        print("Reached marker")
        return marker_info[1]
    
    else:
        # move forward until marker is detected
        chassis_ctrl.move_with_speed(0.4)
        return 0

def get_nearest_marker():
    marker = vision_ctrl.get_marker_detection_info()
    print(marker)
    if len(marker) <= 1:
        return None

    marker_count = marker[0]
    largest_width = 0
    nearest_marker = []

    for i in range(0, marker_count):
        base = 1 + i * 5

        marker_id = marker[base]
        x = marker[base + 1]
        y = marker[base + 2]
        w = marker[base + 3]
        h = marker[base + 4]

        print("Marker:", marker_id)
        print("Width:", w)

        if w > largest_width:
            largest_width = w
            nearest_marker = [marker_id, x, y, w, h]

    return nearest_marker

def moveForwardUntilWall(threshold):
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    
    while distance > threshold:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        chassis_ctrl.move_with_speed(0.5, 0, 0)
        print("Distance: ", distance)
        time.sleep(0.05)
        
    chassis_ctrl.stop()

# -------------------------- claw helper functions --------------------------

def grab():
    while (not gripper_ctrl.is_closed()):
        gripper_ctrl.close()
    
    print("Picked")

def release():
    while (not gripper_ctrl.is_open()):
        gripper_ctrl.open()
    
    print("Dropped")

def track_and_pickup():
    global picked_up
    gripper_ctrl.open()
    # inch forward until IR distance is minimum
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    print("Distance from cone: ", distance)
    while distance > 20:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        chassis_ctrl.move_with_speed(0.1, 0, 0)
        print(distance)
    chassis_ctrl.stop()
    print("picking up")
    grab()
    picked_up = True
    chassis_ctrl.move_with_distance(90, 0.3) # reverse - TUNE VALUE
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 135)

# --------------------- PATH FOLLOWING ---------------------

def navigate():
    # robot should have already entered and turned to face the right alley
    time.sleep(0.2)
    gimbal_ctrl.recenter()
    set_led(0, 255, 0)
    chassis_ctrl.move_with_speed(3, 0, 0) # WAHOOO
    time.sleep(0.8)
    chassis_ctrl.stop() # stop rush

    set_led(255, 255, 0)
    endGrey = False

    while not endGrey:
        marker_id = track_marker()
        print("Marker ID:", marker_id) 

        if marker_id == 11: # marker 1
            turn_left()
        elif marker_id == 12: # marker 2
            turn_right() 
        elif marker_id == 13: # marker 3 - blue path back
            turn_left()
        elif marker_id == 47: # qn mark
            track_and_pickup()
            endGrey = True

def navigate_grey_ir():
    # robot should have already entered and turned to face the right alley
    time.sleep(0.2)
    gimbal_ctrl.recenter()
    set_led(0, 255, 0)
    chassis_ctrl.move_with_speed(3, 0, 0) # WAHOOO
    time.sleep(0.8)
    chassis_ctrl.stop() # stop rush

    set_led(255, 255, 0)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)

def navigate_all():
    endBlue = False

    while not endBlue:
        marker_id = track_marker()
        print("Marker ID:", marker_id) 

        if marker_id == 11: # marker 1
            turn_left()
        elif marker_id == 12: # marker 2
            turn_right() 
        elif marker_id == 13 or marker_id == 14: # marker 3 n 4- blue path back
            turn_left()
        elif marker_id == 8: # heart
            heart()
        elif marker_id == 47: # qn mark
            track_and_pickup()

def navigate_blue_ir():
    # facing marker 3, some distance away
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
    moveForwardUntilWall(wallDistThreshold) # reached heart

# ---------------------------- Marker Handlers ----------------------------

def turn_left():
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)

def turn_right():
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)

# def coral():
#     # need to locate the coral here!
#     # marker 47, qn mark
#     track_and_pickup()
#     grab()
#     pass

def heart():
    if not endRun:
        chassis_ctrl.rotate_with_degree(rm_define.clockwise, 180)
        chassis_ctrl.move_with_distance(90, 0.5) # strafe right to base
        release()
        chassis_ctrl.move_with_distance(-90, 0.2) # strafe left to alley, ready to navigate grey
    else:
        chassis_ctrl.move_with_distance(-90, 0.5) # strafe right to base
        release()


# ---------------------------- MAIN ----------------------------

def main():
    global coral_count
    global endRun

    initialise()
    # TODO: ENTER MAZE, FACE ALLEY
    # add here
    
    while not endRun:
        navigate_all()

main()

