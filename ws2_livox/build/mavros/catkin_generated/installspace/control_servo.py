import argparse
from adafruit_servokit import ServoKit
from time import sleep

def gpio_servo_command(SK, servo_ids, angle):
    """
    Triggers one or more servos using i2c protocol via adafruit_servokit.
    servo_ids: list of servo indices (0?3)
    angle: angle of the servo(s) (0 to close, 180 to open)
    """
    if angle == 0:
        print("Resetting servo(s)")
    elif angle == 180:
        print("Dropping payload")
    else:
        print("Unknown angle command")

    for servo_x in servo_ids:
        print(f"Setting servo {servo_x} to {angle}°")
        SK.servo[servo_x].angle = angle
    sleep(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Trigger servo(s) via I2C using Adafruit ServoKit.")
    parser.add_argument("servo", help="Servo index (0-3) or 'all'")
    parser.add_argument("angle", type=int, help="Servo angle (0-180)")

    args = parser.parse_args()
    SK = ServoKit(channels=16)

    if args.servo == "all":
        servo_indices = list(range(4))
    else:
        try:
            servo_index = int(args.servo)
            if servo_index < 0 or servo_index > 3:
                raise ValueError("Servo index must be between 0 and 3")
            servo_indices = [servo_index]
        except ValueError:
            raise ValueError("Servo must be an integer between 0 and 3, or 'all'")

    gpio_servo_command(SK, servo_indices, args.angle)