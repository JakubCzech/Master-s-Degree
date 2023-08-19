#!/bin/bash
xhost +
export SIMULATION="True"
file=$1
echo $file
docker compose -f compose_dev.yaml kill simulation navigation 
docker compose -f compose_dev.yaml rm -f simulation navigation 
cp $file /home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml
echo "  #############################   Starting $file   #############################"
for i in {1..4}
do
    sed -i "s/Kp: .*/Kp: $i.0/" /home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml
    sed -i "s/Ka: .*/Ka: $((i*2)).0/" /home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml
    for j in $(seq 0.75 0.25 1.55)
    do
        sed -i "s/lookahead_dist: .*/lookahead_dist: $j/" /home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml
        for z in $(seq 0.1 0.1 0.9)
        do 
            echo "  #############################   Test Ka: $((i*2)).0 Kp: $i.0 Lookahead: $j rotate_to_heading_min_angle: $z  #############################" 
            
            sed -i "s/rotate_to_heading_min_angle: .*/rotate_to_heading_min_angle: $z/" /home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml

            docker compose -f compose_dev.yaml kill simulation navigation 
            docker compose -f compose_dev.yaml rm -f simulation navigation 
            docker compose -f compose_dev.yaml up -d simulation navigation 
            sleep 5

            docker exec navigation bash -c "source install/setup.bash && ros2 launch agv_waypoint_sender waypoint_sender.launch.py filename:=$file"
            sleep 2
            echo "Ka: $((i*2)).0 Kp: $i.0 Lookahead: $j rotate_to_heading_min_angle: $z \n" >> /home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/tests_results/test_final.txt

        done
    done
done
echo "  #############################   End of testing $file    #############################"

docker compose -f compose_dev.yaml kill simulation navigation 
docker compose -f compose_dev.yaml rm -f simulation navigation 

