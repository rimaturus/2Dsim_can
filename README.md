**perception:**     g++ perception.cpp -o perception -lyaml-cpp
**pid_control:**    g++ pid_control.cpp -o pid_control -lyaml-cpp -lpthread -lstdc++fs
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



docker run -it \
        --gpus all \
        --user ubuntu \
        --network=host \
        --ipc=host \
        -v /home/edo/test/psd24_simulator/psd_ws:/home/ubuntu/psd_ws \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --env=DISPLAY -v /dev:/dev \
        --device-cgroup-rule="c *:* rmw" \
        --name psd_test test \
        /bin/bash
