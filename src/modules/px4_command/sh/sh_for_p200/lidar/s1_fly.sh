##激光雷达自主飞行脚本
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun bluesea bluesea_node _frame_id:=horizontal_laser_link _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch cartographer_ros amov.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_command move; exec bash"' \

