#!/bin/bash
#$ cat 2rrt_test_100.sh
# Execute 100 birrt tests
echo "Starting 100 birtt tests"


for i in {1..100..1}
do
roslaunch siar_planner rrt_test_in_complex_scenario.launch &
echo "Concluded test $i"
sleep 8
done

echo "Finished 100 birtt tests"

#!/bin/bash
