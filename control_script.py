# control_script.py

import can
from pynput import keyboard  # Using pynput to avoid needing root privileges
import time

# Create a bus instance
bus = can.interface.Bus(channel='vcan0', interface='socketcan')

# Key mapping
key_codes = {'a': 0, 'd': 1, 'w': 2, 's': 3}

# Function to send CAN messages
def send_can_message(can_id, data):
    msg = can.Message(arbitration_id=can_id, data=[data], is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError:
        print("Message not sent")

# Key press event handlers
def on_press(key):
    try:
        if key.char in key_codes:
            send_can_message(0x100, key_codes[key.char])  # Key Pressed
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in key_codes:
            send_can_message(0x101, key_codes[key.char])  # Key Released
    except AttributeError:
        pass
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Start listening
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    print("Control script is running. Press ESC to exit.")
    listener.join()
