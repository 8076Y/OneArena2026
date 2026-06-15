documentation can be found at https://www.dji.com/sg/robomaster-s1/programming-guide

training package: https://drive.google.com/drive/folders/1GdPvwqAJcHHJWAeqxQEZMMZcUPQQJIz8?usp=sharing

this is coded in robomaster app, not python sdk, so a normal python file will kill itself when you try running it

<h2>quick documentation that might be needed during comps</h2>

<h3>set movement speed</h3>

`chassis_ctrl.set_rotate_speed(speed)`
`chassis_ctrl.set_trans_speed(speed)`
- speed: m/s

> just use their library's movement functions, simple and works. speed already configured in initialise() - as much as possible, move slowly to prevent drift on the mecanum wheels 

<h3>rotation</h3>

`chassis_ctrl.rotate_with_degree(direction, angle)`
- direction: `rm_define.clockwise` or `rm_define.anticlockwise`
    - relative to current robot heading
- angle: deg (0-360)

<h3>move (distance)</h3>

`chassis_ctrl.move_with_distance(angle, distance)`
- angle: degree (-180/180)
- distance: meters

<h3>move (tank)</h3>

`chassis_ctrl.move_with_speed(y, x, turn)`

- y: forward speed (m/s), 
- x: right speed (m/s)
- turn: rotation speed (deg/s). ignore unless swerving

> x and y speed are with respect to gimbal heading (ts is different from imu) so remember to reset gimbal, see below

<h3>reset gimbal</h3>

`gimbal_ctrl.recenter()`

<h3>move arm (absolute)</h3>

`robotic_arm_ctrl.moveto(x_coordinate, y_coordinate)` 
- x: horizontal distance, forwards positive (mm)
- y: vertical distance, upwards positie (mm)

<h3>move arm (relative)</h3>

`robotic_arm_ctrl.move(x_coordinate, y_coordinate)`
- x: horizontal distance, forwards positive (mm)
- y: vertical distance, upwards positie (mm)

<h3>distance from wall</h3>

`ir_distance_sensor_ctrl.get_distance_info(1)`
- for some reason this bs is in cm while general mvoement is in m

<h3>led for debugging</h3>

`set_led(R, G, B)`
- this is a custom function

<h3>wait</h3>

`time.sleep(delay)`
- delay: time in s

<h2>reminders</h2>

- tune initial camera (arm) position if mission requires tracking markers. ideal position is when the marker is not cut off from the frame (undetectable when not a full image)
- consider adding a 1s delay after initialisation if there is a marker at an angle from the bot upon start. robot might use the value captured during initialization when the arm is still moving
