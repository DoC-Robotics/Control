import brickpi3  # import the BrickPi3 drivers
import time

# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP = brickpi3.BrickPi3()

try:
	try:
		BP.offset_motor_encoder(
            BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))  # reset encoder A
		BP.offset_motor_encoder(
            BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))  # reset encoder D
	except IOError as error:
		print(error)

	while True:
		try:
			BP.set_motor_power(BP.PORT_A + BP.PORT_D, 12)
			# BP.set_motor_power(BP.PORT_D, 12)
			time.sleep(5)
			BP.set_motor_power(BP.PORT_A + BP.PORT_D, 0)
			# BP.set_motor_power(BP.PORT_D, 0)
			time.sleep(1)
		except KeyboardInterrupt:
			print("failed to run motor")
			break
			time.sleep(2)
		
		print(" Motor A Status: ", BP.get_motor_status(BP.PORT_A), " Motor D Status: ", BP.get_motor_status(BP.PORT_D))

except KeyboardInterrupt:
    BP.reset_all()
    print("failed to run motor")
