docker run -it --user wall_e --env=DISPLAY=:0 --network=host --ipc=host -v /home/edo/2Dsim_can/docker/../:/home/wall_e/sim/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /dev:/dev --privileged --name robotica_ubuntu22 2dsim_image /bin/bash
