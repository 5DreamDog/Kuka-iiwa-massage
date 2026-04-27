#!/usr/bin/env python3


import threading
import time
import os
import rospy
import socket
from std_msgs.msg import String


def cl_red(msg): return '\033[91m' + msg + '\033[0m'
def cl_green(msg): return '\033[92m' + msg + '\033[0m'
def cl_yellow(msg): return '\033[93m' + msg + '\033[0m'
def cl_cyan(msg): return '\033[96m' + msg + '\033[0m'
def cl_pink(msg): return '\033[95m' + msg + '\033[0m'

#   Class: Kuka iiwa TCP communication    #####################
SERVER_IP = '172.31.1.50'
SERVER_PORT = 1234
BUFFER_SIZE = 2048


#   Class: Kuka iiwa TCP communication    #####################
class KukaServer:
	def __init__(self, ip, port):
		self.ip = ip
		self.port = port
		self.isconnected = False
		self.isready = False
		self.connection = None
		self.command_sub = None
	
	
    	# ====== Data from Robot ======
		self.data = {
			'JointPosition': ([None]*7, None),
			'ToolPosition': ([None]*6, None),
			'ToolForce': ([None]*3, None),
			'ToolTorque': ([None]*3, None),
			'isCompliance': (False, None),
			'isCollision': (False, None),
			'isReadyToMove': (False, None),
			'isMastered': (False, None),
			'OperationMode': (None, None),
			'JointAcceleration': (None, None),
			'JointVelocity': (None, None),
			'JointJerk': (None, None),
			'isFinished': (False, None),
			'hasError': (False, None),
		}
    	
    	
    	# Ros publishers
		self.pubs = {}
		print(cl_cyan(f"Debug: __init__ complited"))
		
		# Starting socket in other thread
		thread = threading.Thread(target = self.socket_loop, daemon = True)
		thread.start()
		print(cl_cyan(f"Debug: socket loop thread started"))
    	
	def socket_loop(self):
		# Main loop for connection to robot
		print(cl_cyan(f"Debug: socket loop entered"))
		
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		
		try:
			sock.bind((self.ip, self.port))
			print(cl_cyan(f'Server started on {self.ip}:{self.port}'))
		except Exception as e:
			print(cl_red(f'ERROR: Cannot bind to {self.ip}:{self.port} - {e}'))
			os._exit(-1)
		
		sock.listen(1)
		print(cl_yellow('Waiting for KUKA connection ...'))
		
		self.connection, addr = sock.accept()
		print(cl_green(f'Connected to KUKA: {addr}'))
		
		self.connection.settimeout(0.1)
		self.isconnected = True
		#print(cl_cyan(f'Debug: isconnected = {self.isconnected}'))
		
		
		last_read_time = time.time()
		buffer = ''
		
		#print(cl_cyan(f"Debug: Entering while loop"))
        
		while self.isconnected:
			try:
				#print(cl_cyan(f"Debug: Waiting for data..."))
				chunk = self.connection.recv(BUFFER_SIZE)
				#print(cl_cyan(f"Debug: chunk received: {len(chunk)} bytes"))
				
				if not chunk:
					#print(cl_yellow("Debug: empty chunk, continue"))
					continue
					
				
				# Decode bytes to stringi
				chunk = chunk.decode('utf-8', errors = 'ignore')
				#print(cl_cyan(f"Debug: Received: {repr(chunk[:100])}"))
				
				last_read_time = time.time()
				buffer += chunk
				
				while '\n' in buffer:
					line, buffer = buffer.split('\n', 1)
					line = line.strip()
					if line:
						#print(f"Debug: Parsing: {line[:50]}")
						self.parse_line(line)
						
					if time.time() - last_read_time > 5.0:
						print(cl_red('No data from KUKA for 5 sec!'))
						self.isconnected = False

			except socket.timeout:
				continue           
			except Exception as e:
				print(f"ERROR while loop:{e}")
				import traceback
				traceback.print_exc()
				self.isconnected = False
         
		#print(cl_red(f"Debug: Exiting while loop"))
		self.connection.close()
		sock.close()
		print(cl_red('Connection closed'))
         
	def parse_line(self, line):
		# Parsing string from robot
		if not line.startswith('>'):
			return
			
		line = line[1:]
		parts = line.split(' ', 1)
		
		if len(parts) < 2:
			return
		
		cmd = parts[0]
		value_str = parts[1]
		timestamp = time.time()
		
		try:
			if cmd == 'Joint_Pos':
				values = [float(x.strip()) for x in value_str.split(',')]
				if len(values) == 7:
					self.data['JointPosition'] = (values, timestamp)
					
			elif cmd == 'Tool_Pos':
				values = [float(x.strip()) for x in value_str.split(',')]
				if len(values) == 6:
					self.data['ToolPosition'] = (values, timestamp)
					
			elif cmd == 'Tool_Force':
				values = [float(x.strip()) for x in value_str.split(',')]
				if len(values) == 3:
					self.data['ToolForce'] = (values, timestamp)
					
			elif cmd == 'Tool_Torque':
				values = [float(x.strip()) for x in value_str.split(',')]
				if len(values) == 3:
					self.data['ToolTorque'] = (values, timestamp)
			
			elif cmd in ['isCompliance', 'isCollision', 'isReadyToMove', 'isMastered', 'isFinished', 'hasError']:
				value = value_str.strip().lower() == 'true'
				self.data[cmd] = (value, timestamp)
				
			elif cmd == 'OperationMode':
				self.data['OperationMode'] = (value_str.strip(), timestamp)
				self.isready = True
				#print(cl_green('All data received, is ready = True'))
				
			elif cmd in ['JointAcceleration', 'JointVelocity', 'JointJerk']:
				value = float(value_str.strip())
				self.data[cmd] = (value, timestamp)
				
		except Exception as e:
			print(cl_red(f'Parse error for {cmd}: {e}'))
				
	def publish_loop(self):
		while not self.isready:
			time.sleep(0.1)
		
		self.pubs = {
		'JointPosition': rospy.Publisher('JointPosition', String, queue_size=10),
		'ToolPosition': rospy.Publisher('ToolPosition', String, queue_size=10),
		'ToolForce': rospy.Publisher('ToolForce', String, queue_size = 10),
		'ToolTorque': rospy.Publisher('ToolTorque', String, queue_size = 10),
		'isCompliance': rospy.Publisher('isCompliance', String, queue_size = 10),
		'isCollision': rospy.Publisher('isCollision', String, queue_size = 10),
		'isReadyToMove': rospy.Publisher('isReadyToMove', String, queue_size = 10),
		'isMastered': rospy.Publisher('isMastered', String, queue_size = 10),
		'OperationMode': rospy.Publisher('OperationMode', String, queue_size = 10),
		'JointAcceleration': rospy.Publisher('JointAcceleration', String, queue_size = 10),
		'JointVelocity': rospy.Publisher('JointVelocity', String, queue_size = 10),
		'JointJerk': rospy.Publisher('JointJerk', String, queue_size = 10),
		'isFinished': rospy.Publisher('isFinished', String, queue_size = 10),
		'hasError': rospy.Publisher('hasError', String, queue_size = 10),
	}
		self.command_sub = rospy.Subscriber('kuka_command', String, self.command_callback)
		print(cl_green("Subscribed to kuka command topic"))

		rate = rospy.Rate(10)
		
		while not rospy.is_shutdown() and self.isconnected:
			for topic_name, publisher in self.pubs.items():
				value, ts = self.data[topic_name]
				if isinstance(value, list):
					data_str = '[' + ','.join([str(x) for x in value]) + ']' + str(ts)
				else:
					data_str = str(value) + ' ' + str(ts)
				publisher.publish(data_str)
			rate.sleep()
    
	def command_callback(self, data):
		command = data.data
		print(cl_cyan(f"DEBUG: Recieved command: {command}"))
		self.send_command(command)

	def send_command(self, command):
		if self.connection:
			try:
				self.connection.sendall((command + '\r\n').encode('utf-8'))
				print(cl_green(f"Command sent: {command}"))
			except Exception as e:
				print(cl_red(f"ERROR sending command: {e}"))
		else:
			print(cl_red("ERROR: No connection to robot"))
			
if __name__ == '__main__':
	print(cl_pink('\n==========================='))
	print(cl_pink('     KUKA  SERVER'))
	print(cl_pink('===========================\n'))

	rospy.init_node('kuka_server', anonymous = False)

	server = KukaServer(SERVER_IP, SERVER_PORT)

	thread = threading.Thread(target = server.publish_loop, daemon=True)
	thread.start()

	try:
		while not rospy.is_shutdown():
			time.sleep(0.1)
	except KeyboardInterrupt:
		print(cl_red('\nShutting down...'))
		server.isconnected = False
                                    
                
                
                
                
                
                
                
                
            
            
            
            
    	
    	
    	
    	
    	
    	
