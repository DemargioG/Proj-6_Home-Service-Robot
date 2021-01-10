# /bin/sh

xterm -e " source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 10
xterm -e " source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 7
xterm -e " rosrun add_markers add_markers" &
sleep 5
xterm -e " rosrun pick_objects pick_objects"

