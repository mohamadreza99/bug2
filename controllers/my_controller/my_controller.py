"""my_controller_simple controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from Users.apple.Applications.Webots.app.lib.controller import Robot
from controller import Robot
import numpy as np
from controller import InertialUnit
import math

Threshold = 0.005


def calculate_line_err(distance):
    # print("distance is : ", distance)
    return distance < Threshold


def calculate_distance(a, b, c):
    x0, _, y0 = a
    x1, _, y1 = b
    x2, _, y2 = c
    return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))


def calculate_endCond(current_position, destination):
    return np.sqrt((current_position[0] - destination[0]) ** 2 + (current_position[2] - destination[2]) ** 2)


def detect_obstacle(prox_sensors):
    for i in range(len(prox_sensors)):
        if i == 3 or i == 4:
            continue
        if prox_sensors[i].getValue() > 80:
            return True
    return False


def refine_current_orientation(sensed_orientation, five_prev_orientations):
    if abs(abs(np.average(five_prev_orientations)) - abs(sensed_orientation)) < 0.4:
        return sensed_orientation
    else:
        # print("refined to ", five_prev_orientations[0], " sensed was :", sensed_orientation)
        return five_prev_orientations[0]


def update_prev_orientations_list(new_value, five_prev_orientations):
    for i in range(0, 4):
        five_prev_orientations[i + 1] = five_prev_orientations[i]
    five_prev_orientations[0] = new_value
    return five_prev_orientations


def find_desired_orientation(start_point, destination):
    if destination[0] - start_point[0] < 0:
        if destination[2] - start_point[2] < 0:
            desired = np.arctan((destination[0] - start_point[0]) / (destination[2] - start_point[2]))
        else:
            desired = np.pi - np.arctan(-(destination[0] - start_point[0]) / (destination[2] - start_point[2]))
    else:
        if destination[2] - start_point[2] < 0:
            desired = np.arctan((destination[0] - start_point[0]) / (destination[2] - start_point[2]))
        else:
            desired = np.arctan((destination[0] - start_point[0]) / (destination[2] - start_point[2])) - np.pi

    return desired


def euclid_distance(point1, point2):
    distance = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[2] - point2[2]) ** 2)
    # print(distance)
    return distance


def run_robot(robot):
    # get the time step of the current world.
    timestep = 32
    max_speed = 6.28
    # max speed is 2pi

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    gps = robot.getDevice('gps')
    gps.enable(timestep)

    # enable proximity sensorts ps0 to ps7
    prox_sensors = []
    for i in range(8):
        sensor_name = 'ps' + str(i)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[i].enable(timestep)

    start_point = (-0.75, 0, -0.019)
    destination = (0.5, 0, -0.013)

    inertial = robot.getDevice("inertial unit")
    inertial.enable(timestep)

    mode = 0

    side_detected = False
    right_side_sensors = False

    last_hit_is_valid = False

    five_prev_orientations = [0, 0, 0, 0, 0]
    last_hit = [float('inf'), float('inf'), float('inf')]
    while robot.step(timestep) != -1:
        gps_val = gps.getValues()
        left_wall = prox_sensors[5].getValue() > 80
        right_wall = prox_sensors[2].getValue() > 80
        front_left_wall = prox_sensors[7].getValue() > 80 or prox_sensors[6].getValue() > 80
        front_right_wall = prox_sensors[0].getValue() > 80 or prox_sensors[1].getValue() > 80

        left_speed = 0
        right_speed = 0

        current_orientation = refine_current_orientation(inertial.getRollPitchYaw()[2], five_prev_orientations)
        five_prev_orientations = update_prev_orientations_list(current_orientation, five_prev_orientations)

        desired = find_desired_orientation(start_point, destination)

        # print(gps_val, last_hit)
        if last_hit_is_valid and euclid_distance(gps_val, last_hit) < 0.03:
        	pass
            # print("unreachable")
            # left_speed = 0
            # right_speed = 0
            # mode = -1

        elif calculate_endCond(gps_val, destination) < 0.15:
            left_speed = 0
            right_speed = 0

        elif mode == 0:
            mode = 1

        elif mode == 1:
            # print(desired, current_orientation)
            last_hit_is_valid = True
            if abs(desired - current_orientation) > 0.05:
                left_speed = max_speed
                right_speed = - max_speed
            else:
                mode = 2
        # move forward
        elif mode == 2:
            if detect_obstacle(prox_sensors):
                left_speed = 0
                right_speed = 0
                last_hit = gps_val
                last_hit_is_valid = False
                print("wall detected")
                mode = 3
            else:
                left_speed = max_speed
                right_speed = max_speed
        elif mode == 3:

            # hit the line
            if calculate_line_err(calculate_distance(gps_val, start_point, destination)) and euclid_distance(
                    destination, last_hit) > euclid_distance(destination, gps_val):
                print("line found")
                mode = 1
            else:

                if not side_detected and front_left_wall:
                    print("front wall left -> turn left in place")
                    side_detected = True
                    right_side_sensors = True
                    left_speed = -max_speed
                    right_speed = max_speed
                elif not side_detected and front_right_wall:
                    print("front wall right -> turn right in place")
                    side_detected = True
                    right_side_sensors = False
                    left_speed = max_speed
                    right_speed = -max_speed
                elif side_detected and not right_side_sensors and (front_right_wall or front_left_wall):
                    print("front wall right -> turn right in place")
                    left_speed = max_speed
                    right_speed = -max_speed
                elif side_detected and right_side_sensors and (front_right_wall or front_left_wall):
                    print("front wall left -> turn left in place")
                    left_speed = -max_speed
                    right_speed = max_speed

                elif left_wall or right_wall:
                    last_hit_is_valid = True
                    print("drive forward")
                    left_speed = max_speed
                    right_speed = max_speed
                else:
                    if right_side_sensors:
                        print("turn right")
                        right_speed = max_speed / 8
                        left_speed = max_speed
                    else:
                        print("turn left")
                        right_speed = max_speed
                        left_speed = max_speed / 8

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    robot = Robot()
    print(robot.getCustomData())
    run_robot(robot)
