# Initial attempt to clone the ur_description repo and use it to simulate the UR5 robot in pybullet
# mkdir -p src
# git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b humble src/ur_description

# This repo contains a working example of the UR5 robot in pybullet
git clone git@github.com:ElectronicElephant/pybullet_ur5_robotiq.git src/ur5_pybullet/pybullet_ur5_robotiq

# Install dependencies
# Install pyenv and set it up the .bashrc and .profile
# Install python 3.8.6 so that ur5 works -- attrdict is not compatible with python 3.10
pyenv install 3.8.6
pyenv local 3.8.6
python3 -m venv env

source ./env/bin/activate
pip install torch opencv-python numpy attrdict pybullet tqdm scipy
