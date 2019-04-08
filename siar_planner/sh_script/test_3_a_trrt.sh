#!/bin/bash
#$ cat 2rrt_test_100.sh
# Execute 100 birrt tests

for i in {1..10..1}
do
roslaunch siar_planner test_3_a_trrt_in_fork.launch &
echo "Concluded test $i"
sleep 10
done

echo "Finished 100 trtt tests"

#!/bin/bash
