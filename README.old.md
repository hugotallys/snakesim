# SnakeSim

steps installing snakesim

install ros2 on ubuntu 22.04 (debian packagaes)

mkdir -p ros2_ws/src

clone repo at ros2_ws/src

colcon build --symlink-install

source install/local_setup.zsh

install webots
install webots_ros2_driver

setup python (create requirements)

# Make a virtual env and activate it

virtualenv -p python3 ./venv
source ./venv/bin/activate

# Make sure that colcon doesn’t try to build the venv

touch ./venv/COLCON_IGNORE

install matplotlib, pandas numpy roboticstoolbox-python

pip install -r requirements
