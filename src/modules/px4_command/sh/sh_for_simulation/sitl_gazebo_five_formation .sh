##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch px4_command five_uav_mavros_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun px4_command formation_control_sitl; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun px4_command move; exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun px4_command formation_change; exec bash"' \

