#!/bin/bash


# shellcheck source=src/util.sh
source ~/.bashrc

sudo apt-get update
sudo apt-get upgrade

sudo apt install --assume-yes python3.12-venv

python3 -m venv ~/venv --system-site-packages --symlinks
# shellcheck source=src/util.sh
source ~/venv/bin/activate

pip install --no-input ultralytics

cd ~/ros2_ws/ || exit

colcon build