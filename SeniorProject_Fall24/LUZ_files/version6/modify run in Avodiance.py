def run():
    global turn
    global speed
    global __isRunning
    global HWSONAR
    global Threshold
    global distance_data
    global stopMotor
    global forward
    global old_speed

    dist = HWSONAR.getDistance() / 10.0  # Convert mm to cm
    distance_data.append(dist)
    if len(distance_data) > 5:
        distance_data.pop(0)

    # Calculate smoothed distance
    distance = np.mean(distance_data)

    if __isRunning:
        if distance <= Threshold:  # Obstacle detected
            if turn:
                turn = False
                forward = True
                stopMotor = True
                chassis.set_velocity(0, 90, -0.5)  # Turn slightly
                time.sleep(0.5)
        else:  # Path clear
            if forward:
                turn = True
                forward = False
                stopMotor = True
                chassis.set_velocity(speed, 90, 0)  # Move forward
    else:
        if stopMotor:
            stopMotor = False
            chassis.set_velocity(0, 0, 0)  # Stop all motors
        turn = True
        forward = True
        time.sleep(0.03)

    return distance
