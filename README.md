# enpm809e_summer22_final

# Clone this repository in your catkin workspace

Assuming you have the catkin workspace `catkin_ws` located in your home directory:
- `cd ~/catkin_ws/src`
- `git clone https://github.com/zeidk/enpm809e_summer22_final.git`
- `cd ~/catkin_ws`
- `rosdep install --from-paths . --ignore-src --rosdistro noetic -y` (replace `noetic` with `melodic` if you are using `melodic`)
- `catkin build`
- `source ~/catkin_ws/devel/setup.bash` (add this line in your `.bashrc`)
- `export GAZEBO_MODEL_PATH=absolute path to/summer809e_final/workcell_809e/models:$GAZEBO_MODEL_PATH` (add this line to `.bashrc`)
  - Example: `export GAZEBO_MODEL_PATH=/home/zeid/enpm809e_lecture9_ws/src/summer809e_final/workcell_809e/models:$GAZEBO_MODEL_PATH`

- `roslaunch workcell_809e workcell.launch`
