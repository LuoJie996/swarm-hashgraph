for ((i=0;i<6;i++))
do
docker cp epuck_avoid.py robot$i:/root/catkin_ws/src/epuck_driver_cpp/src/
done
