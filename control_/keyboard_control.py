# control_script.py

import can
import struct
import threading
import time
from pynput import keyboard

# CAN IDs for steering and throttle
STEERING_CAN_ID = 0x300
THROTTLE_CAN_ID = 0x301

# Limits for steering and throttle
MAX_STEERING_ANGLE = 30.0  # degrees
MIN_STEERING_ANGLE = -30.0 # degrees
MAX_THROTTLE = 100.0        # arbitrary units
MIN_THROTTLE = 0.0          # arbitrary units

# Increment steps
STEERING_STEP = 1.0  # degrees per key press
THROTTLE_STEP = 1.0  # units per key press

# Current control states
current_steering = 0.0  # degrees
current_throttle = 0.0  # units

# Lock for thread-safe updates
lock = threading.Lock()

# Flag to indicate when to stop the threads
stop_flag = False

def send_float(can_bus, can_id, value):
    """Sends a float value over CAN."""
    # Pack the float into 4 bytes (little-endian)
    data = struct.pack('<f', value)
    msg = can.Message(arbitration_id=can_id,
                      data=data,
                      is_extended_id=False)
    try:
        can_bus.send(msg)
        # Uncomment the following line for debugging
        # print(f"Sent CAN ID: {hex(can_id)}, Value: {value}")
    except can.CanError as e:
        print(f"Failed to send CAN message: {e}")

def on_press(key, can_bus):
    """Handles key press events."""
    global current_steering, current_throttle, stop_flag
    try:
        if key.char.lower() == 'a':
            with lock:
                current_steering -= STEERING_STEP
                if current_steering < MIN_STEERING_ANGLE:
                    current_steering = MIN_STEERING_ANGLE
                send_float(can_bus, STEERING_CAN_ID, current_steering)
        elif key.char.lower() == 'd':
            with lock:
                current_steering += STEERING_STEP
                if current_steering > MAX_STEERING_ANGLE:
                    current_steering = MAX_STEERING_ANGLE
                send_float(can_bus, STEERING_CAN_ID, current_steering)
        elif key.char.lower() == 'w':
            with lock:
                current_throttle += THROTTLE_STEP
                if current_throttle > MAX_THROTTLE:
                    current_throttle = MAX_THROTTLE
                send_float(can_bus, THROTTLE_CAN_ID, current_throttle)
        elif key.char.lower() == 's':
            with lock:
                current_throttle -= THROTTLE_STEP
                if current_throttle < MIN_THROTTLE:
                    current_throttle = MIN_THROTTLE
                send_float(can_bus, THROTTLE_CAN_ID, current_throttle)
        elif key.char.lower() == 'q':
            print("Quitting control script...")
            stop_flag = True
            return False  # Stop the listener
    except AttributeError:
        # Handle special keys if needed
        pass

def on_release(key, can_bus):
    """Handles key release events."""
    # Optional: Implement gradual return to neutral on key release
    pass

def reset_controls(can_bus):
    """Resets steering and throttle to neutral positions when no keys are pressed."""
    global current_steering, current_throttle, stop_flag
    while not stop_flag:
        # This function can be expanded to implement gradual resetting if desired
        time.sleep(0.1)

def main():
    global stop_flag
    # Initialize CAN bus
    try:
        can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
    except Exception as e:
        print(f"Failed to initialize CAN interface: {e}")
        return

    print("Control Script Started.")
    print("Controls:")
    print("  A: Steer Left")
    print("  D: Steer Right")
    print("  W: Throttle Up")
    print("  S: Throttle Down")
    print("  Q: Quit")

    # Start the listener in a separate thread
    listener = keyboard.Listener(
        on_press=lambda key: on_press(key, can_bus),
        on_release=lambda key: on_release(key, can_bus)
    )
    listener.start()

    try:
        while not stop_flag:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")
        stop_flag = True

    # Stop the listener
    listener.stop()

    # Reset steering and throttle to neutral before exiting
    send_float(can_bus, STEERING_CAN_ID, 0.0)
    send_float(can_bus, THROTTLE_CAN_ID, 0.0)

    print("Control Script Terminated.")

if __name__ == "__main__":
    main()
