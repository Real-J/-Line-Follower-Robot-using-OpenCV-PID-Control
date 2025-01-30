import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Motor GPIO Pins
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
RIGHT_MOTOR_FORWARD = 22
RIGHT_MOTOR_BACKWARD = 23

# PID Controller Variables
Kp = 0.6  # Proportional Gain
Ki = 0.0  # Integral Gain
Kd = 0.2  # Derivative Gain

previous_error = 0
integral = 0

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)

# Initialize PWM
pwm_left = GPIO.PWM(LEFT_MOTOR_FORWARD, 100)
pwm_right = GPIO.PWM(RIGHT_MOTOR_FORWARD, 100)
pwm_left.start(0)
pwm_right.start(0)

def set_motor_speed(left_speed, right_speed):
    """Set motor speed based on PID output"""
    left_speed = max(min(left_speed, 100), 0)  # Constrain values
    right_speed = max(min(right_speed, 100), 0)

    pwm_left.ChangeDutyCycle(left_speed)
    pwm_right.ChangeDutyCycle(right_speed)

def process_frame(frame):
    """Process the camera frame to detect the line"""
    global previous_error, integral

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold to detect black line
    _, threshold = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

    # Get the center of mass of the black region
    M = cv2.moments(threshold)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])  # X coordinate of the centroid
    else:
        cx = frame.shape[1] // 2  # Default to center if no line detected

    # Calculate error
    frame_center = frame.shape[1] // 2
    error = cx - frame_center

    # PID Calculation
    integral += error
    derivative = error - previous_error
    output = (Kp * error) + (Ki * integral) + (Kd * derivative)
    previous_error = error

    # Motor Speed Adjustment
    base_speed = 50  # Adjust as needed
    left_speed = base_speed - output
    right_speed = base_speed + output

    set_motor_speed(left_speed, right_speed)

    return threshold  # Return threshold image for debugging

# Initialize Camera
cap = cv2.VideoCapture(0)  # Use USB Camera or Pi Camera

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        threshold_image = process_frame(frame)
        
        # Display processed image (for debugging)
        cv2.imshow("Threshold", threshold_image)
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopping Robot...")

finally:
    cap.release()
    cv2.destroyAllWindows()
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
