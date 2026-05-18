#
//  template.py
//  OneArena2026
//
//  Created by Yap Han Yang on 23/4/26.
//

# PID Controllers for speed and angular correction
speed_pid = PIDCtrl()         # Varies speed as the robot approaches the marker
angular_pid = PIDCtrl()       # Corrects robot's angle to face the marker

# Flag to indicate end of run and break out of main loop
global endRun
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
    
    # Enable line tracking
    # vision_ctrl.line_follow_color_set(rm_define.line_follow_color_red) # Track red line

    # Enable IR distance sensor
    ir_distance_sensor_ctrl.enable_measure(1)

    # Reset arm and gripper to initial positions (arm down, gripper open)
    robotic_arm_ctrl.recenter(wait_for_complete=True)
    gripper_ctrl.open()
    robotic_arm_ctrl.moveto(200, -70, wait_for_complete=True)

# ---------------------------- Helper Functions ----------------------------

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
    print('Line Track')
    # vision_ctrl.set_line_following_speed(100)
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
    
# ---------------------------- Maze Part Code ----------------------------

def climbRamp():
    """
    Function for climbing the ramp using pitch detection.
    """
    pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
    
    while pitch > -5:
        pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
        chassis_ctrl.move_with_speed(2.0, 0, 0) # x axis (m/s), y axis(m/s), speed roatation(ignore unless swerving
        
    while pitch < -20:
        pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
        chassis_ctrl.move_with_speed(0.1, 0, 0)
        
    chassis_ctrl.stop()

# ---------------------------- Marker Handlers ----------------------------

#def marker1():
#    """
#    Marker 1: Decision point for route selection
#    """
#    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
#    moveForwardUntilWall(30)
#    
#    # ROUTE SELECTION!!!
#    # route1() # Uncomment to use route 1
#    route2()

# ---------------------------- Route 1 ----------------------------

#def route1():
#    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 80)
#    moveForwardUntilWall(30)
#    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 80)
#    moveForwardUntilWall(10)
#    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90)
#    moveForwardUntilWall(30)
#
#def marker2():
#    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90)
#    chassis_ctrl.move_with_distance(-90, 0.15)
#    moveForwardUntilWall(15)
#    
#    # Since there are 2 marker 2s, marker2() will be called TWICE
#    # i.e. robot will turn right and move forward TWICE
#
#def marker3():
#    chassis_ctrl.move_with_distance(90, 0.2) # Strafe right
#    moveForwardUntilWall(30)
#
#def marker4():
#    global endRun
#    
#    chassis_ctrl.move_with_distance(-90, 0.3) # Strafe left
#    chassis_ctrl.move_with_distance(0, 1) # Exit maze
#    
#    # End of run
#    endRun = True
#    print("End of run")

# ---------------------------- Route 2 ----------------------------

#def route2():
#    chassis_ctrl.rotate_with_degree(rm_define.clockwise, 20)
#    chassis_ctrl.move_with_distance(-90, 1) # Strafe left
#    moveForwardUntilWall(30)
#
#def marker5():
#    global endRun
#    
#    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 20)
#    moveForwardUntilWall(30)
#    chassis_ctrl.move_with_distance(90, 1) # Strafe right
#    
#    # End of run
#    endRun = True
#    print("End of run")

# ---------------------------- Main Execution ----------------------------

def main():
    initialise()
    
    # ---------------------------- If Navigating Maze Without Markers ----------------------------
    
    # startingUsingInfrared() # Uncomment 1 only
    # startingUsingDist() # Uncomment 1 only
    # climbRamp()
    
    # ---------------------------- If Navigating Maze With Markers ----------------------------

#    while not endRun:
#        """
#        End the program from one of the marker functions.
#        """
#        markerID = track_marker()
#        markerID -= 10  # Adjust ID range from 10–19 to 0–9
#        print("Marker ID:", markerID)
#
#        if markerID == 1:
#            marker1()
#        elif markerID == 2:
#            marker2()
#        elif markerID == 3:
#            marker3()
#        elif markerID == 4:
#            marker4()
#        elif markerID == 5:
#            marker5()

main()
