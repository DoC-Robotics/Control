# import control
import math
import time
import brickpi3
import test_map
import likelihood
import normalising_resampling
import particlesMCL
import statistics
import math
import random
import histogram

BP = brickpi3.BrickPi3()
BP.reset_all()
left_motor = BP.PORT_B
right_motor = BP.PORT_C
ultrasonic_sensor = BP.PORT_2
BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

NUMBER_OF_PARTICLES = 100
NUM_OF_BINS = 10
rot_scale = 220.0 / 90.0  # per 1 degree #218
straight_scale = 640.0 / 40.0  # per 1 cm
map = likelihood.initialise_map()


def calc_waypoint(target, current):
    """
    target [Wx,Wy]
    current [x,y,theta]
    """
    x_diff = target[0] - current[0]
    y_diff = target[1] - current[1]
    print("----In Waypoint---- x_diff: ", x_diff)

    euclid_distance = math.sqrt(y_diff**2 + x_diff**2)
    angle_diff = test_map.get_ang_diff(euclid_distance, x_diff, y_diff, target, current)
    rotate_amount = angle_diff - current[2]

    # swing to adjust between pi and -pi.
    if rotate_amount < -180:
        rotate_amount += 360

    elif rotate_amount > 180:
        rotate_amount -= 360

    print("----Exit Waypoint-----: ")
    return rotate_amount, rot_scale, euclid_distance, straight_scale


def move_rover(rotate_amount, euclid_distance, particles):
    # update particle weghts

    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)
    rotate_rover(rotate_amount, particles)

    print("Finished Rotating")
    move_rover_forward(euclid_distance, particles)
    print("Finished Moving Forward")
    return particles


def check_within_range(value, target):
    range_min = abs(target) * 0.9
    range_max = abs(target) * 1.1
    return range_min - 5 <= abs(value) <= range_max + 5


def rotate_rover(rotate_amount, particles):
    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)
    try:
        BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
        BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))
    except IOError as error:
        print(error)
    print("rotation START")
    BP.set_motor_position(left_motor, rotate_amount * rot_scale)
    BP.set_motor_position(right_motor, -rotate_amount * rot_scale)
    particles.genNewParticlesRotation(rotate_amount)
    while (
        BP.get_motor_encoder(left_motor) != rotate_amount * rot_scale
        and BP.get_motor_encoder(right_motor) != -rotate_amount * rot_scale
    ):
        if check_within_range(
            BP.get_motor_encoder(left_motor), rotate_amount * rot_scale
        ) and check_within_range(
            BP.get_motor_encoder(right_motor), -rotate_amount * rot_scale
        ):
            break
        continue
    # update particle weghts
    distance_measurement = distance_measured()
    particles = likelihood.update_particles_weights(
        particles, distance_measurement, map
    )
    particles = normalising_resampling.normalising_and_resampling(particles)
    print("rotation END")


def move_rover_forward(euclid_distance, particles):
    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)
    try:
        BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
        BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))
    except IOError as error:
        print(error)
    if euclid_distance > 20:
        euclid_distance = 20
    print("Move Forward Start")
    BP.set_motor_position(left_motor + right_motor, euclid_distance * straight_scale)
    while (
        BP.get_motor_encoder(left_motor) != euclid_distance * straight_scale
        and BP.get_motor_encoder(right_motor) != euclid_distance * straight_scale
    ):
        if check_within_range(
            BP.get_motor_encoder(left_motor), euclid_distance * straight_scale
        ) and check_within_range(
            BP.get_motor_encoder(right_motor), euclid_distance * straight_scale
        ):
            break
        continue
    particles.genNewParticlesStraight(euclid_distance)
    # update particle weghts
    distance_measurement = distance_measured()
    particles = likelihood.update_particles_weights(
        particles, distance_measurement, map
    )
    particles = normalising_resampling.normalising_and_resampling(particles)
    print("Move Forward End")


def distance_measured():
    readings = []
    length_rover = 7.5
    # target_distance = 20 #cm
    ultrasonic_sensor = BP.PORT_2
    BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    while len(readings) < 5:
        try:
            ultrasonicState = BP.get_sensor(ultrasonic_sensor)
            readings.append(ultrasonicState + length_rover)
            print(
                "ultrasonic reading: ",
                ultrasonicState + length_rover,
            )
        except brickpi3.SensorError as error:
            BP.reset_all()
            ultrasonic_sensor = BP.PORT_2
            BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    if len(readings) == 0:
        return distance_measured()
    else:
        median_reading = statistics.median(readings)
        print("Real reading to the facing wall: ", median_reading)
        # NO OBSTACLE AVOIDANCE, AS NOT NEEDED IN THE GIVEN PATH
        # median_reading only used for the updating of MCL particles
        return median_reading


def estimated_position_and_orientation(particles):
    est_x, est_y, est_theta = 0, 0, 0

    # circular mean operation to deal with wrapping of coordinates
    sum_cos = 0
    sum_sin = 0

    for i in range(NUMBER_OF_PARTICLES):
        est_x += particles.coordinates[i][0] * particles.weights[i]
        est_y += particles.coordinates[i][1] * particles.weights[i]

        sum_cos = (
            math.cos(particles.coordinates[i][2] * math.pi / 180) * particles.weights[i]
        )
        sum_sin = (
            math.sin(particles.coordinates[i][2] * math.pi / 180) * particles.weights[i]
        )

    mean_angle = math.atan2(sum_sin, sum_cos)
    est_theta = mean_angle * 180 / math.pi
    return est_x[0], est_y[0], est_theta


def find_initial_position(particles, ls, signature, path):
    # for i in range(15):
    #     distance_measurement = distance_measured()
    #     ls.sig[i] = distance_measurement
    #     likelihood.update_particles_weights(particles, distance_measurement, map)
    #     particles = normalising_resampling.normalising_and_resampling(particles)
    #     rotate_rover(24, particles)
    index, particles, ls = histogram.recognize_location(ls, signature, particles, map)
    # FILL IN: COMPARE ls_read with ls_obs and find the best match

    return index, particles, ls


def find_initial_orientation(particles):
    for i in range(5):
        rotate_rover(72, particles)
        particles.genNewParticlesRotation(72)
        distance_measurement = distance_measured()
        likelihood.update_particles_weights(particles, distance_measurement, map)

        particles = normalising_resampling.normalising_and_resampling(particles)
    return particles


if __name__ == "__main__":
    map.draw()
    start_flag = True
    path = [
        (84, 30),
        (180, 30),
        (180, 54),
        (138, 54),
        (138, 168),
    ]  # path=[(180,54),(138,54),(138,168),(114,168),(114,84),(84,84),(84,30)]
    particles = particlesMCL.particlesMCL()

    particles.initalise_challenge(path)
    ls = histogram.LocationSignature()
    signature = histogram.SignatureContainer()
    initialise_index, particles, ls = find_initial_position(
        particles, ls, signature, path
    )
    # particles.initialise_at(path[initialise_index][0], path[initialise_index][1])
    orientation = histogram.find_orientation(signature.read(initialise_index), ls)
    particles.new_initialization(
        path[initialise_index][0], path[initialise_index][1], orientation
    )
    # particles = find_initial_orientation(particles)
    particles.printParticles(particles.convertNPtoTuples(particles))
    particles.printPaths(particles, map)
    path_index = initialise_index
    print("STARTING AT: ", path_index)

    ###### LOOPING THROUGH THE GIVEN PATH
    while True:
        try:
            x_goal, y_goal = (
                path[(path_index + 1) % 5][0],
                path[(path_index + 1) % 5][1],
            )

            print("X_goal, y_goal, Path Index", x_goal, y_goal, (path_index + 1) % 5)
            # update particle weghts
            distance_measurement = distance_measured()
            particles = likelihood.update_particles_weights(
                particles, distance_measurement, map
            )
            particles = normalising_resampling.normalising_and_resampling(particles)
            # move rober if angle paths deviates too much
            x, y, theta = estimated_position_and_orientation(particles)
            (
                rotate_amount,
                rot_scale,
                euclid_distance,
                straight_scale,
            ) = calc_waypoint([x_goal, y_goal], [x, y, theta])
            if abs(x - x_goal) < 5 and abs(y - y_goal) < 5:
                print("REACHED Point", x_goal, y_goal)
                time.sleep(3)
                path_index += 1
                continue
            particles = move_rover(rotate_amount, euclid_distance, particles)
            particles.printParticles(particles.convertNPtoTuples(particles))
            particles.printPaths(particles, map)

        except KeyboardInterrupt:
            BP.reset_all()
