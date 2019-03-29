#!/bin/bash
#$ cat 2rrt_test_100.sh
# Execute 100 birrt tests
echo "Starting 100 birtt tests"


for i in {1..100..1}
do
roslaunch siar_planner birrt_test_in_fork.launch &
echo "Concluded test $i"
sleep 4
done

echo "Finished 100 birtt tests"

#!/bin/bash
