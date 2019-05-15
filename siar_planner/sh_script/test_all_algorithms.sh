#!/bin/bash

echo "Starting with TESTS to every algorithms in differents scenarios and cases problems"

# export SLEEP_TIME_1=1
# export SLEEP_TIME_2=2

# roslaunch siar_planner test_1_straight_section_map.launch &
# sleep 2


#echo "Starting test_1_a_rrt"




for i in {1..3..1}
do
  # roslaunch siar_planner test.launch planner_type:=rrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  roslaunch siar_planner test.launch planner_type:=trrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  # roslaunch siar_planner test.launch planner_type:=birrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  roslaunch siar_planner test.launch planner_type:=tbirrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  #sleep $SLEEP_TIME_1
done


for i in {4..6..1}
do 
  # roslaunch siar_planner test.launch planner_type:=rrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  roslaunch siar_planner test.launch planner_type:=trrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  # roslaunch siar_planner test.launch planner_type:=birrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  roslaunch siar_planner test.launch planner_type:=tbirrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  #sleep $SLEEP_TIME_1
done

#echo "Starting test_1_a_birrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_1_a_birrt_in_straight_section.launch 
#echo "Concluded test_1_a_birrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_1_a_trrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_1_a_trrt_in_straight_section.launch 
#echo "Concluded test_1_a_trrt, loop $i"
#sleep $SLEEP_TIME_1
#done    

#echo "Starting test_1_a_tbirrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_1_a_tbirrt_in_straight_section.launch 
#echo "Concluded test_1_a_tbirrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_1_b_rrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_1_b_rrt_in_straight_section.launch 
#echo "Concluded test_1_b_rrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_1_b_birrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_1_b_birrt_in_straight_section.launch 
#echo "Concluded test_1_b_birrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_1_b_trrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_1_b_trrt_in_straight_section.launch 
#echo "Concluded test_1_b_trrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_1_b_tbirrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_1_b_tbirrt_in_straight_section.launch 
#echo "Concluded test_1_b_tbirrt, loop $i"
#sleep $SLEEP_TIME_1
#done




#echo "Starting test_2_a_rrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_2_a_rrt_in_fork_straight.launch 
#echo "Concluded test_2_a_rrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_2_a_birrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_2_a_birrt_in_fork_straight.launch 
#echo "Concluded test_2_a_birrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_2_a_trrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_2_a_trrt_in_fork_straight.launch 
#echo "Concluded test_2_a_trrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_2_a_tbirrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_2_a_tbirrt_in_fork_straight.launch 
#echo "Concluded test_2_a_tbirrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_2_b_rrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_2_b_rrt_in_fork_straight.launch 
#echo "Concluded test_2_b_rrt, loop $i"
#sleep $SLEEP_TIME_1
#done

#echo "Starting test_2_b_birrt"
#for i in {1..100..1}
#do
#roslaunch siar_planner test_2_b_birrt_in_fork_straight.launch 
#echo "Concluded test_2_b_birrt, loop $i"
#sleep $SLEEP_TIME_1
#done

# echo "Starting test_2_b_trrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_2_b_trrt_in_fork_straight.launch 
# echo "Concluded test_2_b_trrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_2_b_tbirrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_2_b_tbirrt_in_fork_straight.launch 
# echo "Concluded test_2_b_tbirrt, loop $i"
# sleep $SLEEP_TIME_1
# done






# echo "Starting test_3_a_rrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_a_rrt_in_fork.launch 
# echo "Concluded test_3_a_rrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_3_a_birrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_a_birrt_in_fork.launch 
# echo "Concluded test_3_a_birrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_3_a_trrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_a_trrt_in_fork.launch 
# echo "Concluded test_3_a_trrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_3_a_tbirrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_a_tbirrt_in_fork.launch 
# echo "Concluded test_3_a_tbirrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_3_b_rrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_b_rrt_in_fork.launch 
# echo "Concluded test_3_b_rrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_3_a_birrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_b_birrt_in_fork.launch 
# echo "Concluded test_3_b_birrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_3_b_trrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_b_trrt_in_fork.launch 
# echo "Concluded test_3_b_trrt, loop $i"
# sleep $SLEEP_TIME_1
# done

# echo "Starting test_3_b_tbirrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_3_b_tbirrt_in_fork.launch 
# echo "Concluded test_3_b_tbirrt, loop $i"
# sleep $SLEEP_TIME_1
# done



# roslaunch siar_planner test_map_complex_scenario.launch &
# sleep 2

# echo "Starting test_4_a_rrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_4_a_rrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_a_rrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_4_a_birrt"
# for i in {1..120..1}
# do
# roslaunch siar_planner test_4_a_birrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_a_birrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_4_a_trrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_4_a_trrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_a_trrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_4_a_tbirrt"
# for i in {1..120..1}
# do
# roslaunch siar_planner test_4_a_tbirrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_a_tbirrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_4_b_rrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_4_b_rrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_b_rrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_4_b_birrt"
# for i in {1..120..1}
# do
# roslaunch siar_planner test_4_b_birrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_b_birrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_4_b_trrt"
# for i in {1..100..1}
# do
# roslaunch siar_planner test_4_b_trrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_b_trrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_4_b_tbirrt"
# for i in {1..120..1}
# do
# roslaunch siar_planner test_4_b_tbirrt_in_complex_scenario_turn_L.launch 
# echo "Concluded test_4_b_tbirrt, loop $i"
# sleep $SLEEP_TIME_2
# done






# echo "Starting test_5_a_rrt"
# for i in {1..200..1}
# do
# roslaunch siar_planner test_5_a_rrt_in_complex_scenario_turn_U.launch 
# echo "Concluded test_5_a_rrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_5_a_birrt"
# for i in {1..250..1}
# do
# roslaunch siar_planner test_5_a_birrt_in_complex_scenario_turn_U.launch 
# echo "Concluded test_5_a_birrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_5_a_trrt"
# for i in {1..200..1}
# do
# roslaunch siar_planner test_5_a_trrt_in_complex_scenario_turn_U.launch 
# echo "Concluded test_5_a_trrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_5_a_tbirrt"
# for i in {1..250..1}
# do
# roslaunch siar_planner test_5_a_tbirrt_in_complex_scenario_turn_U.launch 
# echo "Concluded test_5_a_tbirrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# # echo "Starting test_5_b_rrt"
# # for i in {1..100..1}
# # do
# # roslaunch siar_planner test_5_b_rrt_in_complex_scenario_turn_U.launch 
# # echo "Concluded test_5_b_rrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done

# # echo "Starting test_5_b_birrt"
# # for i in {1..100..1}
# # do
# # roslaunch siar_planner test_5_b_birrt_in_complex_scenario_turn_U.launch 
# # echo "Concluded test_5_b_birrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done

# # echo "Starting test_5_b_trrt"
# # for i in {1..100..1}
# # do
# # roslaunch siar_planner test_5_b_trrt_in_complex_scenario_turn_U.launch 
# # echo "Concluded test_5_b_trrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done

# # echo "Starting test_5_b_tbirrt"
# # for i in {1..100..1}
# # do
# # roslaunch siar_planner test_5_b_tbirrt_in_complex_scenario_turn_U.launch 
# # echo "Concluded test_5_b_tbirrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done







# echo "Starting test_6_a_rrt"
# for i in {1..200..1}
# do
# roslaunch siar_planner test_6_a_rrt_in_complex_scenario_turn_fork.launch 
# echo "Concluded test_6_a_rrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_6_a_birrt"
# for i in {1..250..1}
# do
# roslaunch siar_planner test_6_a_birrt_in_complex_scenario_turn_fork.launch 
# echo "Concluded test_6_a_birrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_6_a_rrt"
# for i in {1..200..1}
# do
# roslaunch siar_planner test_6_a_trrt_in_complex_scenario_turn_fork.launch 
# echo "Concluded test_6_a_rrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# echo "Starting test_6_a_tbirrt"
# for i in {1..250..1}
# do
# roslaunch siar_planner test_6_a_tbirrt_in_complex_scenario_turn_fork.launch 
# echo "Concluded test_6_a_tbirrt, loop $i"
# sleep $SLEEP_TIME_2
# done

# # echo "Starting test_6_b_rrt"
# # for i in {1..100..1}
# # do
# # roslaunch siar_planner test_6_b_rrt_in_complex_scenario_turn_fork.launch 
# # echo "Concluded test_6_b_rrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done

# # echo "Starting test_6_b_birrt"
# # for i in {1..120..1}
# # do
# # roslaunch siar_planner test_6_b_birrt_in_complex_scenario_turn_fork.launch 
# # echo "Concluded test_6_b_birrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done

# # echo "Starting test_6_b_trrt"
# # for i in {1..100..1}
# # do
# # roslaunch siar_planner test_6_b_trrt_in_complex_scenario_turn_fork.launch 
# # echo "Concluded test_6_b_trrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done

# # echo "Starting test_6_b_tbirrt"
# # for i in {1..120..1}
# # do
# # roslaunch siar_planner test_6_b_tbirrt_in_complex_scenario_turn_fork.launch 
# # echo "Concluded test_6_b_tbirrt, loop $i"
# # sleep $SLEEP_TIME_2
# # done



# echo "Finished all the TEST to every algorithms in differents scenarios and cases problems"

# #!/bin/bash
