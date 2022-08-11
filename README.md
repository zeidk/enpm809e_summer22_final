# Packages for the final project Summer 2022 (ENPM809E)

# Steps to clone and run packages

Assuming you have the catkin workspace `catkin_ws` located in your home directory:
- `cd ~/catkin_ws/src`
- `git clone https://github.com/zeidk/enpm809e_summer22_final.git`
- `cd ~/catkin_ws`
- `rosdep install --from-paths . --ignore-src --rosdistro noetic -y` (replace `noetic` with `melodic` if you are using `melodic`)
- `catkin build`
- `source ~/catkin_ws/devel/setup.bash` (add this line in your `.bashrc`)
- `export GAZEBO_MODEL_PATH=absolute path to/summer809e_final/workcell_809e/models:$GAZEBO_MODEL_PATH` (add this line in your `.bashrc`)
  - Example: `export GAZEBO_MODEL_PATH=/home/zeid/enpm809e_lecture9_ws/src/summer809e_final/workcell_809e/models:$GAZEBO_MODEL_PATH`
- `source ~/.bashrc`
- `roslaunch workcell_809e workcell.launch`

# Use the correct shebang line

If you are using Python2 and Melodic:
- Change the shebang line `#!/usr/bin/env python3` to `#!/usr/bin/env python` in the files located in `bot_controller/nodes`.
