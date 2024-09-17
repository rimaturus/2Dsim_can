**perception:**     g++ perception.cpp -o perception -lyaml-cpp
**pid_control:**    g++ pid_control.cpp -o pid_control lyaml-cpp -lpthread -lstdc++fs
**visualization:**  g++ visualization.cpp -o visualization -lallegro -lallegro_main -lallegro_font -lallegro_ttf -lallegro_primitives -lyaml-cpp -pthread

**keyboard_control:**   python3 keyboard_control.py
**track_designer:**     python3 track_designer.py   

**can_receiver:**   g++ can_receiver.cpp -o can_receiver
**can_sender:**   g++ can_sender.cpp -o can_sender

## CAN ID:
# 0x200 -> X_car [meters]
# 0x201 -> Y_car [meters]
# 0x202 -> yaw_car [radians]

# 0x300 -> steering angle [degree]
# 0x301 -> pedal position [units]

# 0x(400 + hex(i)) -> detected cone i (4 byte: range [m] + 4 Byte: bearing [rad])


# ToDO
1) Implement a yaml file to load all relevant parameters (car specs, conversione pixel2meters, etc. )
2) Add a more sophisticated vehicle model (hot swappable model option)
3) Make a more modular code