import cv2
import time
import RPi.GPIO as GPIO

# Scene paths
SCENE = "mole_spin"
SAVE_PATH = "../data/in/" + SCENE + "/{id}.jpg"

# GPIO
DIR_PIN = 12
STEP_PIN = 11
ENABLE_PIN = 10     # Active low

GPIO.setmode (GPIO.BCM)
GPIO.setwarnings (False)
GPIO.setup (STEP_PIN, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup (ENABLE_PIN, GPIO.OUT)
GPIO.output (ENABLE_PIN, GPIO.HIGH)

# Motor
TICKS_PER_ROTATION = 200
STEP_DELAY = 0.002

# Camera
cap = cv2.VideoCapture (0)
CAM_DELAY = 0.1

# Sweep
for tick in range (TICKS_PER_ROTATION):
    # Capture
    cap.read ()
    cv2.imwrite (SAVE_PATH.format (id = tick))
    time.sleep (CAM_DELAY)

    # Rotate
    GPIO.output (STEP_PIN, GPIO.HIGH)
    time.sleep (STEP_DELAY)
    GPIO.output (STEP_PIN, GPIO.LOW)
    time.sleep (STEP_DELAY)