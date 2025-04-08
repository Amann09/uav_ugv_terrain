from controller import Robot
import mavic2proHelper
from simple_pid import PID
import csv
import struct

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

params = dict()
with open("../params.csv", "r") as f:
	lines = csv.reader(f)
	for line in lines:
		params[line[0]] = line[1]

TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = 3.1415926535897932384626433

robot = Robot()

[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = mavic2proHelper.getMotorAll(robot)

timestep = int(robot.getBasicTimeStep())
mavic2proMotors = mavic2proHelper.getMotorAll(robot)
mavic2proHelper.initialiseMotors(robot, 0)
mavic2proHelper.motorsSpeed(robot, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY)

front_left_led = robot.getLED("front left led")
front_right_led = robot.getLED("front right led")
gps = robot.getGPS("gps")
gps.enable(TIME_STEP)
imu = robot.getInertialUnit("inertial unit")
imu.enable(TIME_STEP)
compass = robot.getCompass("compass")
compass.enable(TIME_STEP)
gyro = robot.getGyro("gyro")
gyro.enable(TIME_STEP)


yaw_setpoint=-1

pitchPID = PID(float(params["pitch_Kp"]), float(params["pitch_Ki"]), float(params["pitch_Kd"]), setpoint=0.0)
rollPID = PID(float(params["roll_Kp"]), float(params["roll_Ki"]), float(params["roll_Kd"]), setpoint=0.0)
throttlePID = PID(float(params["throttle_Kp"]), float(params["throttle_Ki"]), float(params["throttle_Kd"]), setpoint=1)
yawPID = PID(float(params["yaw_Kp"]), float(params["yaw_Ki"]), float(params["yaw_Kd"]), setpoint=float(yaw_setpoint))

targetX, targetY, target_altitude = 0.0, 0.0, 10.0

while (robot.step(timestep) != -1):

	led_state = int(robot.getTime()) % 2
	front_left_led.set(led_state)
	front_right_led.set(int(not(led_state)))

	roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
	pitch = imu.getRollPitchYaw()[1]
	yaw = compass.getValues()[0]
	roll_acceleration = gyro.getValues()[0]
	pitch_acceleration = gyro.getValues()[1]
	# print(gyro.getValues())
	
	xGPS = gps.getValues()[2]
	yGPS = gps.getValues()[0]
	zGPS = gps.getValues()[1]

	vertical_input = throttlePID(zGPS)
	print(f"vertical input: {vertical_input}")
	yaw_input = yawPID(yaw)
	print(f"yaw input: {yaw_input}")

	rollPID.setpoint = targetX
	pitchPID.setpoint = targetY
	
	roll_input = float(params["k_roll_p"]) * roll + roll_acceleration + rollPID(xGPS)
	pitch_input = float(params["k_pitch_p"]) * pitch - pitch_acceleration + pitchPID(-yGPS)

	front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input - pitch_input + yaw_input
	front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input - pitch_input - yaw_input
	rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input + pitch_input - yaw_input
	rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input + pitch_input + yaw_input
	
	# front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - yaw_input + pitch_input - roll_input
           # front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + yaw_input + pitch_input + roll_input
           # rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input + yaw_input - pitch_input - roll_input
           # rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input - yaw_input - pitch_input + roll_input

	mavic2proHelper.motorsSpeed(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)