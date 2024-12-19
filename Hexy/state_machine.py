import serial
import math
import time
import threading
import numpy as np
from HexyColorDetect import getDirections
from multiprocessing import Process, Queue

# Constants for inverse kinematics
J2L = 50.0  # Length of J2 (50 mm)
J3L = 50.5  # Length of J3 (50.5 mm)
Y_Rest = -18
Z_Rest = 75

# Servo IDs
front_right = 24
front_left = 7
middle_right = 20
middle_left = 11
back_right = 16
back_left = 15

# Serial configuration
SERIAL_PORT = '/dev/ttyACM0'  # Change this to your port
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

#bajs

servo_offsets = {
	5:150, 6:0, 7:0, 9:-100, 10:0, 11:0, 13:50 , 14:150, 15:0, 16:0, 17:150, 18:100, 20:0, 21:50, 22:150, 24:0, 25:100, 26:50
}


# Helper function to map angles to PWM values
def map_to_pwm(angle):
	return int((angle - 0) * (2500 - 500) / (180 - 0) + 500)


# Check and limit angles to valid range (0 - 180)
def check_range(angle):
	if angle < 0:
		angle = 0
	elif angle > 180:
		angle = 180
	return angle


def cubic_bezier(p0, p1, p2, p3, t):
	x = (1-t)**3 * p0[0] + 3 * (1-t)**2 * t * p1[0] + 3 * (1-t) * t**2 * p2[0] + t**3 * p3[0]
	y = (1-t)**3 * p0[1] + 3 * (1-t)**2 * t * p1[1] + 3 * (1-t) * t**2 * p2[1] + t**3 * p3[1]
	z = (1-t)**3 * p0[2] + 3 * (1-t)**2 * t * p1[2] + 3 * (1-t) * t**2 * p2[2] + t**3 * p3[2]
	return x, y ,z

# Might not use
def quadratic_bezier(p0, p1, p2, t):
	#P0: Starting point, P1: Control point, P2: End point
	x = (1-t)**2 * p0[0] + 2 * (1-t) * t * p1[0] + t**2 * p2[0]
	y = (1-t)**2 * p0[1] + 2 * (1-t) * t * p1[1] + t**2 * p2[1]
	z = (1-t)**2 * p0[2] + 2 * (1-t) * t * p1[2] + t**2 * p2[2]
	return x, y ,z


def linear_bezier(p0, p2, t):
	x = p0[0] + t * (p2[0] - p0[0])
	y = p0[1] + t * (p2[1] - p0[1])
	z = p0[2] + t * (p2[2] - p0[2])
	return x, y ,z
	
# Might not use	
def ease_in_out(p0, p2, t):
	progress = (1 - math.cos(t * math.pi)) / 2
	x = p0[0] + (p2[0] - p0[0]) * progress
	y = p0[1] + (p2[1] - p0[1]) * progress
	z = p0[2] + (p2[2] - p0[2]) * progress
	
	return x, y ,z

# Function to send servo commands over serial
def send_servo_commands(batch_positions):
	command = ""
	for i in range(len(batch_positions)):
		servo_id = batch_positions[i][0]
		pos1 = batch_positions[i][1]
		pos2 = batch_positions[i][2]
		pos3 = batch_positions[i][3]
		if (servo_id > 15):
			pos1 += servo_offsets.get(servo_id, 0)
			pos2 += servo_offsets.get(servo_id+1, 0)
			pos3 += servo_offsets.get(servo_id+2, 0)
			command += f"#{servo_id+2:02d}P{pos3}\n #{servo_id+1:02d}P{pos2}\n #{servo_id:02d}P{pos1}\n"
		else:
			pos1 += servo_offsets.get(servo_id, 0)
			pos2 += servo_offsets.get(servo_id-1, 0)
			pos3 += servo_offsets.get(servo_id-2, 0)
			#print(servo_offsets.get(servo_id-2, 0))
			command += f"#{servo_id-2:02d}P{pos3}\n #{servo_id-1:02d}P{pos2}\n #{servo_id:02d}P{pos1}\n"
			
	ser.write(command.encode())
	

# Function to move a leg using inverse kinematics
def inverse_kinematic(leg_id, point):
	# Calculate J1, J2, J3 usiservo_offsetsng inverse kinematics
	X = point[0]
	Y = point[1]
	Z = point[2]
	
	#print(f"Point: {X, Y, Z}")
	
	if leg_id == front_right:
		J1 = (90 - 27) + math.degrees(math.atan(X / Y))
	elif leg_id == middle_right:
		J1 = (90) + math.degrees(math.atan2(X, Y))
	elif leg_id == back_right:
		J1 = 90 + math.degrees(math.atan(X / Y))
	elif leg_id == front_left:
		J1 = (90 + 27) - math.degrees(math.atan(X / Y))
	elif leg_id == middle_left:
		J1 = (90) - math.degrees(math.atan2(X , Y))
	elif leg_id == back_left:
		J1 = (90 - 27) + math.degrees(math.atan(X / Y))

	H = math.sqrt(X**2 + Y**2) # 2d x and y , hypothenusis
	L = math.sqrt(H**2 + Z**2)
	if L > J2L + J3L:
		print("leg position out of range")
		exit()
	#print(f"H: {H}")
	#print(f"L: {L}")
	#print(f"(2 * J2L * J3L): {(2 * J2L * J3L)}")
	#print(f"(J2L**2 + J3L**2 - L**2): {(J2L**2 + J3L**2 - L**2)}")
	#print((J2L**2 + J3L**2 - L**2) / (2 * J2L * J3L))
	J3 = math.degrees(math.acos((J2L**2 + J3L**2 - L**2) / (2 * J2L * J3L))) - 90
	J2 = math.degrees(math.atan(Z / H)) + 90

	J1, J2, J3 = check_range(J1), check_range(J2), check_range(J3)

	# Interpolation for smooth movement
	pos1, pos2, pos3 = map_to_pwm(J1), map_to_pwm(J2), map_to_pwm(J3)
	
	#print(f"Sending command: {J1}, {J2}, {J3}")
	
	return leg_id, pos1, pos2, pos3
	#send_servo_command(leg_id, pos1, pos2, pos3)

def stand():
	t_values = np.linspace(0,1,1)

	# p0_m = [-14, 58, 75] # with degree = 27/2
	p0_m = [0, 60, 75] # with degree = 27

	p0 = [0, 60, 75]
	
	p0_br = [0, 66, 75]
	
	p0_bl = [34, 52, 75]

	
	
	#Calculate the back movement of the legs
	linear_points = [linear_bezier(p0, p0, t) for t in t_values]
	linear_points_m = [linear_bezier(p0_m, p0_m, t) for t in t_values]
	linear_points_br = [linear_bezier(p0_br, p0_br, t) for t in t_values]
	linear_points_bl = [linear_bezier(p0_bl, p0_bl, t) for t in t_values]
	
	batch_positions = []

	batch_positions.append(inverse_kinematic(front_left, linear_points[0]))
	batch_positions.append(inverse_kinematic(middle_right, linear_points_m[0]))
	batch_positions.append(inverse_kinematic(back_left, linear_points_bl[0]))
	batch_positions.append(inverse_kinematic(front_right, linear_points[0]))
	batch_positions.append(inverse_kinematic(middle_left, linear_points_m[0]))
	batch_positions.append(inverse_kinematic(back_right, linear_points_br[0]))
	send_servo_commands(batch_positions)
	batch_positions.clear()
	time.sleep(0.15)
	#time.sleep(1)		
		
	
	


def walk_forward():
	t_values = np.linspace(0,1,3)
	#Move legs Front left, back left, and middle right
	p0_m = [-25, 55, 75]
	p1_m = [18, 110, -50]
	p2_m = [25, 55, 75]
	
	#p0_m = [0, 60, 75]
	#p1_m = [12, 90, -30]
	#p2_m = [26, 85, -30]
	#p3_m = [36, 54, 75]
	
	#p0_m = [0, 75, 60]
	#p1_m = [17.02, 115, -70] # Control point for bezier
	#p2_m = [34.05, 65, 60]
	
	p0 = [0, 60, 75]
	p1 = [17, 115, -40]
	p2 = [34, 55, 75] # z = 75 (planted on the ground)
	
	# br = back right
	p0_br = [0, 66, 75]
	p1_br = [17, 100, -40]
	p2_br = [34, 52, 75]
	#Calculate the front movement of the legs using QUADRATIC bezier
	curve_points = [quadratic_bezier(p0, p1, p2, t) for t in t_values]
	curve_points_m = [quadratic_bezier(p0_m, p1_m, p2_m, t) for t in t_values]
	curve_points_bl = [quadratic_bezier(p2, p1, p0, t) for t in t_values]
	curve_points_br = [quadratic_bezier(p0_br, p1_br, p2_br, t) for t in t_values]
	
	#Calculate the back movement of the legs
	linear_points = [linear_bezier(p2, p0, t) for t in t_values]
	linear_points_m = [linear_bezier(p2_m, p0_m, t) for t in t_values]
	linear_points_bl = [linear_bezier(p0, p2, t) for t in t_values]
	linear_points_br = [linear_bezier(p2_br, p0_br, t) for t in t_values]
	# print("Linear points m", linear_points_m)
	
	batch_positions = []
	batch_positions.clear()
	for i in range(len(curve_points_m)):
		batch_positions.append(inverse_kinematic(front_left, curve_points[i]))
		batch_positions.append(inverse_kinematic(middle_right, curve_points_m[i]))
		batch_positions.append(inverse_kinematic(back_left, curve_points_bl[i]))
		batch_positions.append(inverse_kinematic(front_right, linear_points[i]))
		batch_positions.append(inverse_kinematic(middle_left, linear_points_m[i]))
		batch_positions.append(inverse_kinematic(back_right, linear_points_br[i]))
		#print(batch_positions)
		#print(batch_positions[0][0])
		send_servo_commands(batch_positions)
		batch_positions.clear()
		#time.sleep(0.3)
		time.sleep(0.15)
		#time.sleep(0.02)
		
	#time.sleep(1)
	for i in range(len(linear_points_m)):
		batch_positions.append(inverse_kinematic(front_left, linear_points[i]))
		batch_positions.append(inverse_kinematic(middle_right, linear_points_m[i]))
		batch_positions.append(inverse_kinematic(back_left, linear_points_bl[i]))
		batch_positions.append(inverse_kinematic(front_right, curve_points[i]))
		batch_positions.append(inverse_kinematic(middle_left, curve_points_m[i]))
		batch_positions.append(inverse_kinematic(back_right, curve_points_br[i]))
		send_servo_commands(batch_positions)
		batch_positions.clear()
		time.sleep(0.15)
		#time.sleep(0.3)
		#time.sleep(0.02)
		
	#time.sleep(1)	
	
def turn_right():
	t_values = np.linspace(0,1,3)

	p0_m = [0, 63, 75]
	p1_m = [18, 110, -50]
	p2_m = [36, 53, 75]

	p0 = [0, 60, 75]
	p1 = [17, 115, -40]
	p2 = [34, 55, 75]
	
	p0_br = [0, 66, 75]
	p1_br = [17, 100, -40]
	p2_br = [34, 52, 75]
	
	
	#Calculate the back movement of the legs
	linear_points_set1 = [linear_bezier(p2, p0, t) for t in t_values]
	linear_points_m_set1 = [linear_bezier(p0_m, p2_m, t) for t in t_values]
	linear_points_b_set1 = [linear_bezier(p0, p2, t) for t in t_values]
	
	linear_points_set2 = [linear_bezier(p0, p2, t) for t in t_values]
	linear_points_m_set2 = [linear_bezier(p2_m, p0_m, t) for t in t_values]
	linear_points_b_set2 = [linear_bezier(p2_br, p0_br, t) for t in t_values]
	
	
	
	#Calculate the front movement of the legs using CUBIC bezier
	curve_points_set1 = [quadratic_bezier(p0, p1, p2, t) for t in t_values]
	curve_points_m_set1 = [quadratic_bezier(p2_m, p1_m, p0_m, t) for t in t_values]
	curve_points_b_set1 = [quadratic_bezier(p2, p1, p0, t) for t in t_values]
	
	curve_points_set2 = [quadratic_bezier(p2, p1, p0, t) for t in t_values]
	curve_points_m_set2 = [quadratic_bezier(p0_m, p1_m, p2_m, t) for t in t_values]
	curve_points_b_set2 = [quadratic_bezier(p0_br, p1_br, p2_br, t) for t in t_values]
	
	
	batch_positions = []
	batch_positions.clear()
	for i in range(len(curve_points_m_set1)):
		batch_positions.append(inverse_kinematic(front_left, curve_points_set1[i]))
		batch_positions.append(inverse_kinematic(middle_right, curve_points_m_set1[i]))
		batch_positions.append(inverse_kinematic(back_left, curve_points_b_set1[i]))
		batch_positions.append(inverse_kinematic(front_right, linear_points_set2[i]))
		batch_positions.append(inverse_kinematic(middle_left, linear_points_m_set2[i]))
		batch_positions.append(inverse_kinematic(back_right, linear_points_set2[i]))
		send_servo_commands(batch_positions)
		batch_positions.clear()
		time.sleep(0.15)
	#time.sleep(1)		
		
	
	for i in range(len(linear_points_m_set1)):
		batch_positions.append(inverse_kinematic(front_right, curve_points_set2[i]))
		batch_positions.append(inverse_kinematic(middle_left, curve_points_m_set2[i]))
		batch_positions.append(inverse_kinematic(back_right, curve_points_set2[i]))
		batch_positions.append(inverse_kinematic(front_left, linear_points_set1[i]))
		batch_positions.append(inverse_kinematic(middle_right, linear_points_m_set1[i]))
		batch_positions.append(inverse_kinematic(back_left, linear_points_b_set1[i]))
		send_servo_commands(batch_positions)
		batch_positions.clear()
		time.sleep(0.15)
	#time.sleep(1)

		
def turn_left():
	t_values = np.linspace(0,1,3)

	p0_m = [0, 63, 75]
	p1_m = [18, 110, -50]
	p2_m = [36, 53, 75]

	p0 = [0, 60, 75]
	p1 = [17, 115, -40]
	p2 = [34, 55, 75]
	
	p0_br = [0, 66, 75]
	p1_br = [17, 100, -40]
	p2_br = [34, 52, 75]
	
	
	#Calculate the back movement of the legs
	linear_points_set1 = [linear_bezier(p0, p2, t) for t in t_values]
	linear_points_m_set1 = [linear_bezier(p2_m, p0_m, t) for t in t_values]
	linear_points_b_set1 = [linear_bezier(p2, p0, t) for t in t_values]
	
	linear_points_set2 = [linear_bezier(p2, p0, t) for t in t_values]
	linear_points_m_set2 = [linear_bezier(p0_m, p2_m, t) for t in t_values]
	linear_points_b_set2 = [linear_bezier(p0_br, p2_br, t) for t in t_values]
	
	
	
	#Calculate the front movement of the legs using CUBIC bezier
	curve_points_set1 = [quadratic_bezier(p2, p1, p0, t) for t in t_values]
	curve_points_m_set1 = [quadratic_bezier(p0_m, p1_m, p2_m, t) for t in t_values]
	curve_points_b_set1 = [quadratic_bezier(p0, p1, p2, t) for t in t_values]
	
	curve_points_set2 = [quadratic_bezier(p0, p1, p2, t) for t in t_values]
	curve_points_m_set2 = [quadratic_bezier(p2_m, p1_m, p0_m, t) for t in t_values]
	curve_points_b_set2 = [quadratic_bezier(p2_br, p1_br, p0_br, t) for t in t_values]
	
	
	batch_positions = []
	batch_positions.clear()
	for i in range(len(curve_points_m_set1)):
		batch_positions.append(inverse_kinematic(front_left, curve_points_set1[i]))
		batch_positions.append(inverse_kinematic(middle_right, curve_points_m_set1[i]))
		batch_positions.append(inverse_kinematic(back_left, curve_points_b_set1[i]))
		batch_positions.append(inverse_kinematic(front_right, linear_points_set2[i]))
		batch_positions.append(inverse_kinematic(middle_left, linear_points_m_set2[i]))
		batch_positions.append(inverse_kinematic(back_right, linear_points_set2[i]))
		send_servo_commands(batch_positions)
		batch_positions.clear()
		time.sleep(0.15)
	#time.sleep(1)		
		
	
	for i in range(len(linear_points_m_set1)):
		batch_positions.append(inverse_kinematic(front_right, curve_points_set2[i]))
		batch_positions.append(inverse_kinematic(middle_left, curve_points_m_set2[i]))
		batch_positions.append(inverse_kinematic(back_right, curve_points_set2[i]))
		batch_positions.append(inverse_kinematic(front_left, linear_points_set1[i]))
		batch_positions.append(inverse_kinematic(middle_right, linear_points_m_set1[i]))
		batch_positions.append(inverse_kinematic(back_left, linear_points_b_set1[i]))
		send_servo_commands(batch_positions)
		batch_positions.clear()
		time.sleep(0.15)
	#time.sleep(1)

		
def punch():
	t_values = np.linspace(0,1,20)
	#Move legs Front left, back left, and middle right
	
	p0 = [20, 20, -50]
	# p1 = [17, 115, -40]
	p2 = [60, 60, -50]
	
	
	#Calculate the front movement of the legs using QUADRATIC bezier
	# curve_points = [quadratic_bezier(p0, p1, p2, t) for t in t_values]
	linear_points_f= [linear_bezier(p0, p2, t) for t in t_values]
	
	#Calculate the back movement of the legs
	linear_points_b = [linear_bezier(p2, p0, t) for t in t_values]
	
	batch_positions = []
	batch_positions.clear()
	for i in range(len(linear_points_f)):
		batch_positions.append(inverse_kinematic(front_right, linear_points_f[i]))
		send_servo_commands(batch_positions)
		batch_positions.clear()
		time.sleep(0.01)
		
	for i in range(len(linear_points_b)):
		batch_positions.append(inverse_kinematic(front_right, linear_points_b[i]))
		send_servo_commands(batch_positions)
		batch_positions.clear()
		time.sleep(0.01)
		
	#time.sleep(1)
	#for i in range(len(linear_points)):
	#	batch_positions.append(inverse_kinematic(front_right, curve_points[i]))
	#	send_servo_commands(batch_positions)
	#	batch_positions.clear()
	#	time.sleep(0.15)


def main_control():
	direction_queue = Queue()
	direction_process = Process(target=getDirections, args=(direction_queue,))
	direction_process.start()
	
	last_direction = None
	active_direction = None
	stand_still = False
	
	try:
		while True:
			new_direction = None
			
			while not direction_queue.empty():
				new_direction = direction_queue.get()
				
				if new_direction != last_direction:
					active_direction = new_direction
				
				if active_direction != "punch": # reset stand_still so that it is ready for next time hexy wants to punch 
					stand_still = False
				
			if active_direction == "forward":
				walk_forward()
				print("Walking forward")
			elif active_direction == "left":
				turn_left()
				print("Turning left")
			elif active_direction == "right":
				turn_right()
				print("Turning right")
			elif active_direction == "punch" and not stand_still: # should stand first before starting the punch session
				stand()
				stand_still = True
				print("stand")
			elif stand_still and active_direction == "punch": # punchy time - if it already has stood still 
				punch()
				print("punch")
			elif active_direction == "stop" or active_direction is None:
				print("Stopped, active direction = ", active_direction)
				#stop()
			
			

			#time.sleep(0.1)
			time.sleep(0.1)
	
	except KeyboardInterrupt:
		print("Terminating control...")
		ser.write("K".encode())
		direction_process.terminate()
		
if __name__ == "__main__":
	#walk_thread = threading.Thread(target=tripod_gait_walk)
	#turnL_thread = threading.Thread(target=tripod_gait_turn_L)
	#turnR_thread = threading.Thread(target=tripod_gait_turn_R)

	
	
	#tripod1_thread.start()
	#walk_thread.start()
	#turnL_thread.start()
	#turnR_thread.start()
	#try:
	#	while True:
	#		walk_forward()
		
	#except KeyboardInterrupt:
	#	print("Terminating main...")
		#ser.write("K".encode())
		#direction_process.terminate()
	
	main_control()	
