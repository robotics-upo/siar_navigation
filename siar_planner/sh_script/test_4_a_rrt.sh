#!/bin/bash
#$ cat 2rrt_test_100.sh
# Execute 100 birrt tests

for i in {1..10..1}
do
roslaunch siar_planner test_4_a_rrt_in_complex_scenario_turn_L.launch &
echo "Concluded test $i"
sleep 10
done

echo "Finished 100 rtt tests"

#!/bin/bash
