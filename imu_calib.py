'''
print imu values for calibration
pitch is angle along slope
yaw is left/right rotation on ground
'''

while True:
    pitch = chassis_ctrl.get_attitude(rm_define.chassis_pitch)
    yaw = chassis_ctrl.get_attitude(rm_define.chassis_yaw)
    print(f'Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')
    time.sleep(0.1)
