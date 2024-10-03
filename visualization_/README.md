sudo apt-get update
sudo apt-get install build-essential liballegro5-dev liballegro-ttf5-dev liballegro5-dev libyaml-dev

gcc -o visualization visualization.c -lallegro -lallegro_font -lallegro_ttf -lallegro_primitives -lpthread -lyaml -lm
