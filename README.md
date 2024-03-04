# SnakeSim

# Make a virtual env and activate it
virtualenv -p python3 ./venv
source ./venv/bin/activate
# Make sure that colcon doesn’t try to build the venv
touch ./venv/COLCON_IGNORE

pip install -r requirements



