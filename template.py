# PID Controllers for speed and angular correction
speed_pid = PIDCtrl()         # Varies speed as the robot approaches the marker
angular_pid = PIDCtrl()       # Corrects robot's angle to face the marker

# Global Flags
picked_up = False
endRun = False
"""
Example Usage:

(Inside function)
    global endRun
    endRun = True
    print('End of run')

(Inside main)
    while not endRun:
"""

# -------------------------- Initialisation Function --------------------------

def initialise():
    """
    Default robot initialisation to use in every program.
    """
    # Set PID control parameters
    speed_pid.set_ctrl_params(5, 0, 0)
    angular_pid.set_ctrl_params(150, 0, 0)

    # Set default rotation speed
    chassis_ctrl.set_rotate_speed(100)

    # Enable vision detection for markers (not needed for line tracking)
    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    vision_ctrl.set_marker_detection_distance(3)
    
    # Reset gimbal
    gimbal_ctrl.recenter()
    
    # Enable line tracking
    # vision_ctrl.line_follow_color_set(rm_define.line_follow_color_red) # Track red line

    # Enable IR distance sensor
    ir_distance_sensor_ctrl.enable_measure(1)

    # Reset arm and gripper to initial positions (arm down, gripper open)
    if robotic_arm_ctrl.get_position()[1] < 0: # to prevent arm from jamming if never reset properly
        robotic_arm_ctrl.move(0, 0-robotic_arm_ctrl.get_position()[1], wait_for_complete=True)
    robotic_arm_ctrl.recenter(wait_for_complete=True)
    gripper_ctrl.open()
    robotic_arm_ctrl.move(0, 20, wait_for_complete=True)
    robotic_arm_ctrl.moveto(200, -70, wait_for_complete=True)
    
# ----------------------- Debugging Helper Functions -----------------------

def set_led(R, G, B):
    led_ctrl.set_bottom_led(rm_define.armor_bottom_front, R, G, B, rm_define.effect_always.on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_left, R, G, B, rm_define.effect_always.on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_right, R, G, B, rm_define.effect_always.on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_back, R, G, B, rm_define.effect_always.on)


# ------------------------ Movement Helper Functions ------------------------

def track_marker():
    """
    Adjust robot's position until in front of marker and return marker id.
    Might want to consider having different thresholds for different ids.
    """
    global distance_from_marker
    marker_id = 0
    threshold = 30  # Distance threshold (to be tuned)

    # Get marker information
    marker_info = vision_ctrl.get_marker_detection_info()
    if len(marker_info) > 2:
        marker_id = marker_info[1]
        print(marker_id)

        # Get distance and set PID errors
        distance_from_marker = ir_distance_sensor_ctrl.get_distance_info(1)
        speed_pid.set_error(distance_from_marker)
        x_offset = marker_info[2] - 0.5
        angular_pid.set_error(x_offset)

        # Adjust position until marker is close enough
        while distance_from_marker > threshold:
            chassis_ctrl.move_with_speed(0.5, 0, angular_pid.get_output())
            time.sleep(0.005)

            marker_info = vision_ctrl.get_marker_detection_info()
            distance_from_marker = ir_distance_sensor_ctrl.get_distance_info(1)
            speed_pid.set_error(distance_from_marker)
            x_offset = marker_info[2] - 0.5
            angular_pid.set_error(x_offset)

            print("Distance", distance_from_marker)

        chassis_ctrl.stop()
        print("Reached marker")
        return marker_id
    
    else:
        # Move forward until marker is detected
        chassis_ctrl.move_with_speed(0.4)
        return 0

def moveForwardUntilWall(threshold):
    """
    Moves forward until the IR sensor detects a wall within the threshold distance (cm).
    """
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    
    while distance > threshold:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        chassis_ctrl.move_with_speed(0.5, 0, 0)
        print("Distance: ", distance)
        time.sleep(0.05)
        
    chassis_ctrl.stop()

def lineTrack():
    """
    Follow line until obstacle detected.
    """
    print('Line Track')
    # vision_ctrl.set_line_following_speed(100) # i don't think this is needed
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    media_ctrl.exposure_value_update(rm_define.exposure_value_small)

    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    print('Looping distance')

    while distance > 30:  # Distance to be tuned
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        print("Distance: ", distance)                           # Without obstacle, distance should be ~37
        lineInfo = vision_ctrl.get_line_detection_info()        # 0: 10, 1: Line Type, Sets of 4 index are data of line points
        print(lineInfo)

        if lineInfo[1] == 1:  # 1 means a single line is detected
            x = lineInfo[14]        # Take 4th point to correct to future line changes better
            x_offset = x - 0.5      # Distance of point from centre of bot
            angular_pid.set_error(x_offset)  # Inputs error into PID function
            chassis_ctrl.move_with_speed(0.4, 0, angular_pid.get_output())  # Angular velocity to turn and correct

        else:
            # If no line detected, move forward to search
            chassis_ctrl.move_with_speed(0.3, 0, 0)

        time.sleep(0.05)

    print('End of looping distance')
    vision_ctrl.disable_detection(rm_define.vision_detection_line)

def climbRamp():
    """
    Function for climbing a ramp using pitch detection.
    """
    pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
    
    while pitch > -5:
        pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
        chassis_ctrl.move_with_speed(2.0, 0, 0) # x axis (m/s), y axis(m/s), speed roatation(ignore unless swerving
        
    while pitch < -20:
        pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
        chassis_ctrl.move_with_speed(0.1, 0, 0)
        
    chassis_ctrl.stop()

def arc_turnfn():
    """
    Continuous sweep. MUST TEST AND TUNE.
    """
    gimbal_ctrl.recenter() # for some reason ts relies on gimbal so just reset to zero here
    for i in range(220):
        chassis_ctrl.move_with_speed(0.3, 0.3, 70)
    chassis_ctrl.stop()
    time.sleep(1)
    chassis_ctrl.move_with_distance(-180, 0.5)

# -------------------------- Claw Helper Functions --------------------------

def grab():
    """
    Close the claw.
    """
    while (not gripper_ctrl.is_closed()):
        gripper_ctrl.close() # Close the gripper to pick up the cone
    
    print("Picked")

def release():
    """
    Open the claw.
    """
    while (not gripper_ctrl.is_open()):
        gripper_ctrl.open() # Open the gripper to drop the cone
    
    print("Dropped")

def track_and_pickup():
    global picked_up
    """
    Move forward until object is in claw, then pick up.
    """
    gripper_ctrl.open()
    # Inch forward until IR distance is minimum
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    print("Distance from cone: ", distance)
    while distance > 10:
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        chassis_ctrl.move_with_speed(0.1, 0, 0)
        print(distance)
    chassis_ctrl.stop()
    print("picking up")
    grab()
    robotic_arm_ctrl.moveto(200, -20)
    picked_up = True
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 180)

# Isaac drop function for reference
#def dropfn():
#    global picked_up
#    #stops any ongoing movement of the robot's chassis,
#    #ensuring the robot is stationary before it attempts to drop an object
#    chassis_ctrl.stop()
#    if picked_up == True: #if the robot is loaded
#        #chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 85) #robot turn 85 degree anticlockwise
#        #chassis_ctrl.move_with_distance(0, 0.35) #0.35 in meter
#        robotic_arm_ctrl.moveto(200, -70, wait_for_complete=True)
#        #open the gripper - drop the item
#        release()
#        print("Dropped")
#        picked_up = False
#        robotic_arm_ctrl.recenter() #move robot's arm back to default position
#        chassis_ctrl.move_with_distance(180, 0)
#        chassis_ctrl.rotate_with_degree(rm_define.clockwise, 180)
#    else:
#        print("Nothing to drop")
#    run_time_drop = tools.run_time_of_program()
#    print(run_time_drop)

# --------------------- TO ADAPT IF THERE IS A MAZE ---------------------

def startingUsingInfrared():
    """
    Start navigation using infrared sensing.
    """
    wallDistThreshold = 20  # Distance threshold (to be tuned)

    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
    moveForwardUntilWall(wallDistThreshold)
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)

def startingUsingDist():
    """
    Start navigation using predefined distances. Last resort.
    """
    chassis_ctrl.move_with_distance(0, 2) # 0 deg, 2m
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
    chassis_ctrl.move_with_distance(0, 1)
    chassis_ctrl.move_with_distance(-90, 1)
    chassis_ctrl.move_with_distance(0, 1)
    chassis_ctrl.move_with_distance(90, 1)

# ---------------------------- Marker Handlers ----------------------------

#def marker1():
#    """
#    Marker 1: Decision point for route selection
#    """
#    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
#    moveForwardUntilWall(30)

# ---------------------------- Main Execution ----------------------------

def main():
    initialise()
    
    # ---------------------------- If Navigating Maze Without Markers ----------------------------
    
    # startingUsingInfrared() # Uncomment 1 only
    # startingUsingDist() # Uncomment 1 only
    # climbRamp()
    
    # ---------------------------- If Navigating Maze With Markers ----------------------------

#    while not endRun:
#      marker_id = track_marker()
#      print("Marker ID:", marker_id)
#
#      if marker_id == 0:
#          continue

main()
