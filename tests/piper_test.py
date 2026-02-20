import time
from piper_control import piper_connect

#CAN CONNECTION
print("Available CAN Ports:", piper_connect.find_ports())
piper_connect.activate()
print("Activated CAN Port:", piper_connect.active_ports())

#ARM INIT
from piper_control import piper_interface, piper_init

robot = piper_interface.PiperInterface(can_port="can0")
print("BE SURE THE ARM IS NOT IN TEACHING MODE")
print(f'Robot general state: {robot.show_status()}')
print(f'Robot joint vel: {robot.get_joint_velocities()}')
print(f'Robot joint effort: {robot.get_joint_efforts()}')
print(f'Robot joint position: {robot.get_joint_positions()}')
print(f'ROBOT LIMITS: {robot.joint_limits}')
input("ATTENTION THE ARM WILL DROP: PRESS RETURN TO CONTINUE IF YOU ARE SURE")

piper_init.reset_arm(
    robot,
    arm_controller=piper_interface.ArmController.POSITION_VELOCITY,
    move_mode=piper_interface.MoveMode.JOINT,
)
print("Arm reset completed")

#PRINT INFO
#High level state of the robot
print(f'Robot general state: {robot.show_status()}')
print(f'Robot joint vel: {robot.get_joint_velocities()}')
print(f'Robot joint effort: {robot.get_joint_efforts()}')
print(f'Robot joint position: {robot.get_joint_positions()}')

#MOVE JOINTS
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ideal_pos = [1.5, 0.0, -2.9, 0.0, 0.0, 0.0]
robot.command_joint_positions(joint_angles)
print(f'Robot starting position: {robot.get_joint_positions()}')
robot.command_joint_positions(ideal_pos)
print(f'Robot ideal position: {robot.get_joint_positions()}')

test_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
robot_limits = {'min': [-2.687, 0.0, -3.054, -1.745, -1.309, -1.745], 'max': [2.687, 3.403, 0.0, 1.954, 1.309, 1.745]}
increment = 0.0
joint_test = 0
while True:
    print(f'Robot general state: {robot.show_status()}')
    print(f'Robot joint vel: {robot.get_joint_velocities()}')
    print(f'Robot joint effort: {robot.get_joint_efforts()}')
    print(f'Robot joint position: {robot.get_joint_positions()}')
    joint_test = int(input(f'Select Joint (8 to reset, 7 to task position):  '))

    if joint_test < 5:
        increment = float(input(f'Change Value for Joint{joint_test}:  '))
        if (test_angle[joint_test] + increment) > robot_limits["max"][joint_test] or (test_angle[joint_test] + increment) < robot_limits["min"][joint_test]:
            increment = 0.0
        test_angle[joint_test] = test_angle[joint_test] + increment
        joint_angles[joint_test] = test_angle[joint_test]
        robot.command_joint_positions(joint_angles)

    elif joint_test == 8:
        joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        test_angle = joint_angles
        robot.command_joint_positions(joint_angles)

    elif joint_test == 7:
        test_angle = ideal_pos
        robot.command_joint_positions(ideal_pos)