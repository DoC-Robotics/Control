import brickpi3 # import the BrickPi3 drivers
import time

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3$

try:
	while True:
		BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) 
		BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) 
		print("enter the for loop")
		BP.set_motor_power(BP.PORT_A,10)
		target = BP.get_motor_encoder(BP.PORT_A)
		BP.set_motor_position(BP.PORT_D,target)
		time.sleep(5)
		BP.set_motor_power(BP.PORT_A,0)
		target = BP.get_motor_encoder(BP.PORT_D)
		BP.set_motor_position(BP.PORT_D,target)
		time.sleep(4)

except KeyboardInterrupt:
	print("failed to run motor")






