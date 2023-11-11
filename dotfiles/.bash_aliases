# ln -s ~/anon_auton_ws/dotfiles/.bash_aliases ~/.bash_aliases
# in ~/.bashrc --> change -f to -L when checking for ~/.bash_aliases

alias setup="cd ~/anon_auton_ws && source devel/setup.bash"
alias chair="~/anon_auton_ws/src/launch_manager/scripts/chair_on_startup.bash"
alias killros="~/anon_auton_ws/src/launch_manager/launch/kill_ros.sh"
alias pre="~/anon_auton_ws/src/launch_manager/launch/pre.sh"
alias antenna="setup && roslaunch --wait ~/anon_auton_ws/src/launch_manager/launch/components/antenna_chair.launch"
alias arduino="setup && roslaunch --wait ~/anon_auton_ws/src/launch_manager/launch/components/arduino.launch"
alias camera="setup && roslaunch --wait ~/anon_auton_ws/src/launch_manager/launch/components/camera.launch"
alias lidar="setup && roslaunch --wait ~/anon_auton_ws/src/launch_manager/launch/components/lidar.launch"
alias queue="setup && roslaunch --wait ~/anon_auton_ws/src/launch_manager/launch/components/queue.launch"
alias handwritten="setup && rm -rf /tmp/handwritten-input && mkfifo /tmp/handwritten-input && cat > /tmp/handwritten-input & && echo $! > /tmp/handwritten-input-pid && cd ~/anon_auton_ws && tail -f /tmp/handwritten-input | ./src/launch_manager/launch/launch_handwritten_ros.sh &"

# Broadcast
alias ready="setup && rostopic pub -1 from_chair std_msgs/String Br"
alias success="setup && rostopic pub -1 from_chair std_msgs/String Bs"
alias fail="setup && rostopic pub -1 from_chair std_msgs/String Bf"