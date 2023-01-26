import brickpi3 # import the BrickPi3 drivers
import time

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

while True:
	try:
		BP.set_motor_power(BP.PORT_A,12)
		BP.set_motor_power(BP.PORT_D,12)
		time.sleep(5)
		BP.set_motor_power(BP.PORT_A,0)
		BP.set_motor_power(BP.PORT_D,0)
		time.sleep(1)
	except KeyboardInterrupt:
		print("failed to run motor")
		break
		time.sleep(2)




