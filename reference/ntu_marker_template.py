def start():
    # set robot to free mode
    # enable vision marker detector
    # set vision marker detection distance
    # enable no.1 infrared distance sensor
    # set rotation speed to 45deg/s
    # set translation speed to 0.2m/s
    found=True

    while True:
        found = align_with_marker()
        # What value should be return?
        print("Found:",found)
        if not found:
            print("No more markers")
            break
        
        move_forward()
        rotate()

        time.sleep(0.5)

def align_with_marker():
    global marker_list
    ### initialise a variable "lost_count" to 0

    while True:
        marker_list = vision_ctrl.get_marker_detection_info()
        ### For multiple markers,
        ### marker_list = get_nearest_marker()

        if marker_list is not None:
            ### marker found, set lost_count to 0
            # print("Nearest marker ID:", marker_list[1])
            # x_coord = marker_list[1]
            # x_coord = marker_list[2]
            # error = x_coord - 0.5
            # error = 0.5 - x_coord
            # error = x_coord

            print("x =", x_coord)
            print("error =", error)

            # if abs(error) < 0.07:
            # if abs(error) > 0.07:
                print("Aligned")
                ### marker found and aligned
                ### return a value to continue the while loop

            move_distance = max(abs(error), 0.05)
            if error > 0:
                print("Move left")
                chassis_ctrl.move_with_distance(-90, move_distance)
            else:
                print("Move right")
                chassis_ctrl.move_with_distance(90, move_distance)
            time.sleep(0.5)

        else:
            print("No marker detected")
            ### marker not found, increase lost_count by 1
            time.sleep(0.1)
            ### when lost_count >= 20
                ### return a value to stop the function

def move_forward():
    # enable IR sensor
    # set translation speed
    # move forward (0 degree)
    # while loop
        # assign distance detected to “distance”
        # move until “distance” less than or equal to 10
        # stop
        # exit the loop
    # pause for 0.05 second

def rotate():
    # For example:
    # when marker “3” detected, turn left
    # when marker “4” detected, turn right

    # if “3” detected
        # rotate 90 degree to left
    # else if “4” detected
        # rotate 90 degree to right

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