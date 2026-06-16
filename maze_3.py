# global thresholds to tune
wallDistThreshold = 22
markerThreshold = 22

# pid for marker alignment
speed_pid = PIDCtrl()
angular_pid = PIDCtrl()

# global flags
two_count = 0
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
        if two_count == 5:
            yaw = chassis_ctrl.get_attitude(rm_define.chassis_yaw)
            if 87 < yaw < 93:
                chassis_ctrl.move_with_speed(1) # speedrun to reduce long distance drift
            else:
                chassis_ctrl.stop()
                yaw = chassis_ctrl.get_attitude(rm_define.chassis_yaw)
                if yaw < 90:
                    chassis_ctrl.rotate_with_degree(rm_define.clockwise, int(90-yaw))
                else:
                    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, int(yaw-90))
        else:
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
    robotic_arm_ctrl.move(0, 0-robotic_arm_ctrl.get_position()[1], wait_for_complete=True) # to prevent arm jamming
    robotic_arm_ctrl.recenter(wait_for_complete=False)

# --------------------- PATH FOLLOWING ---------------------

def navigate_all():
    global two_count
    marker_id = track_marker()
    print("Marker ID:", marker_id) 

    if marker_id == 11: # marker 1
        turn_left()
    elif marker_id == 12: # marker 2
        two_count += 1
        turn_right() 
    elif marker_id == 13 or marker_id == 14: # marker 3, 4
        turn_left()
    elif marker_id == 8: # heart
        heart()
    elif marker_id == 47: # qn mark
        coral()

# ---------------------------- Marker Handlers ----------------------------

def turn_left():
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)

def turn_right():
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)

def coral():
    global two_count
    two_count += 1
    robotic_arm_ctrl.moveto(185, -65, wait_for_complete=True)
    track_and_pickup()
    chassis_ctrl.move_with_distance(90, 0.3) # go right
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 150)

def heart():
    global endRun
    global two_count
    global coral_count
    coral_count += 1
    two_count = 0
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 45)
    robotic_arm_ctrl.moveto(200, -70)
    release()
    robotic_arm_ctrl.move(0, 0-robotic_arm_ctrl.get_position()[1], wait_for_complete=True) # to prevent arm jamming
    robotic_arm_ctrl.recenter(wait_for_complete=False)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 135)
    if coral_count == 4:
        endRun = True


# ---------------------------- MAIN ----------------------------

def main():
    global two_count
    global endRun

    initialise()
    
    while not endRun:
        navigate_all()

main()
