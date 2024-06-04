# this makefile assumes that raylib is installed locally and headers are available in /usr/local/include.
# to build raylib on linux, follow instructions at this page:
# https://github.com/raysan5/raylib/wiki/Working-on-GNU-Linux
# sudo apt install libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxcursor-dev libxinerama-dev libwayland-dev libxkbcommon-dev
# sudo make install


# gcc -o bin/sim src/sim.c
# gcc -o bin/sim src/sim.c -I include

# gcc -o bin/sim src/sim.c -I include -I /usr/local/include -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
sim: src/sim.c
	gcc -g -o bin/sim src/sim.c \
		-I include -I /usr/local/include \
		-lraylib -lGL -lm -lpthread -ldl -lrt -lX11
