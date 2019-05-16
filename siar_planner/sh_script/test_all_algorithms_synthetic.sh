#!/bin/bash

echo "Starting with TESTS to every algorithms in differents scenarios and cases problems"

# export SLEEP_TIME_1=1
# export SLEEP_TIME_2=2


for i in {1..3..1}
do
  roslaunch siar_planner test_synthetic.launch planner_type:=rrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  roslaunch siar_planner test_synthetic.launch planner_type:=trrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  roslaunch siar_planner test_synthetic.launch planner_type:=birrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  roslaunch siar_planner test_synthetic.launch planner_type:=tbirrt n_tests:=$1 problem_number:=$i map_number:=1 parameters_number:=1
  #sleep $SLEEP_TIME_1
done


for i in {4..6..1}
do 
  roslaunch siar_planner test_synthetic.launch planner_type:=rrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  roslaunch siar_planner test_synthetic.launch planner_type:=trrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  roslaunch siar_planner test_synthetic.launch planner_type:=birrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  roslaunch siar_planner test_synthetic.launch planner_type:=tbirrt n_tests:=$1 problem_number:=$i map_number:=2 parameters_number:=2
  #sleep $SLEEP_TIME_2
done


 echo "Finished all the TEST to every algorithms in differents scenarios and cases problems"

# #!/bin/bash
