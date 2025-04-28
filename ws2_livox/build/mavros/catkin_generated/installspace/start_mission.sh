cd "/home/uhdt/ws2_livox/"
source "devel/setup.bash"
rosservice call /mavros/set_stream_rate 0 100 1
rosrun mavros auto_test3
