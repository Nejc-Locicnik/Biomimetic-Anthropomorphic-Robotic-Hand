import serial
import threading

ser1 = serial.Serial(port='/dev/ttyUSB0', baudrate=9600)
ser2 = serial.Serial(port='/dev/ttyUSB1', baudrate=9600)
ser3 = serial.Serial(port='/dev/ttyUSB2', baudrate=9600)
ser4 = serial.Serial(port='/dev/ttyUSB3', baudrate=9600)

def recieve_data():
	while True:
		try:
			print("serial01: " + ser1.readLine().decode('UTF-8').replace('\n',''))
			print("serial02: " + ser2.readLine().decode('UTF-8').replace('\n',''))
			print("serial03: " + ser3.readLine().decode('UTF-8').replace('\n',''))
			print("serial04: " + ser4.readLine().decode('UTF-8').replace('\n',''))
		except:
			pass
	
def send_data():
	cmd = input("New command: \n")
	if len(cmd) > 1:
		if cmd[0] == '1':
			ser1.write((cmd[1:]).encode('UTF-8'))
		elif cmd[0] == '2':
			ser2.write((cmd[1:]).encode('UTF-8'))
		elif cmd[0] == '3':
			ser3.write((cmd[1:]).encode('UTF-8'))
		elif cmd[0] == '4':
			ser4.write((cmd[1:]).encode('UTF-8'))
		else:
			print("Wrong command")

if __name__ == "__main__":
	recieve_thread = threading.Thread(target=recieve_data)
	recieve_thread.daemon = True
	recieve_thread.start()

	while True:
		send_data()
    