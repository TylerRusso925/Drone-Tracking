import RPi.GPIO as GPIO
import time
import threading

# Pin numbers
output_pins = [15, 33]

# Function to control PWM on a given pin
def pwm_control(pin, initial_angle, increment):
    # Set pin as an output pin with optional initial state of HIGH
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
    p = GPIO.PWM(pin, 50)  # 50Hz PWM frequency
    angle = initial_angle

    p.start(2.5 + initial_angle/18)  # Start PWM with initial angle

    try:
        while True:
            time.sleep(0.02)
            if angle >= 180 or angle <= 0:
                increment = -increment
            angle += increment
            p.ChangeDutyCycle(2.5 + angle/18)
    finally:
        p.stop()

def main():
    # Pin Setup: Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)

    # Create threads for each pin
    threads = []
    for pin in output_pins:
        thread = threading.Thread(target=pwm_control, args=(pin, 90, 5))
        threads.append(thread)
        thread.start()

    print("PWM running on both pins. Press CTRL+C to exit.")
    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        print("Exiting program.")

    GPIO.cleanup()

if __name__ == '__main__':
    main()
