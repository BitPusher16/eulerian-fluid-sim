# eulerian-fluid-sim

Project based on Ten Minute Physics video here:
https://www.youtube.com/watch?v=iKAVRgIrUOU

## Installing and Running

```
# Makefile assumes that raylib is installed locally and headers are available in /usr/local/include.
# to build raylib on linux, follow instructions at this page:
# https://github.com/raysan5/raylib/wiki/Working-on-GNU-Linux

sudo apt install libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxcursor-dev libxinerama-dev libwayland-dev libxkbcommon-dev

cd ~/repos
git clone git@github.com:raysan5/raylib.git
cd raylib
sudo apt install build-essential git
sudo make install

cd ~/repos
git clone git@github.com:BitPusher16/eulerian-fluid-sim.git
cd eulerian-fluid-sim
make
./bin/sim ./data/simple_wing_160_120.png


# To generate a compile_commands.json file which nvim can read, run:
bear make
```
