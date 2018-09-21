#!/bin/bash

END=8
for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --get_grasping_pose --hand both --remove-outliers "(0.01 10)" --file "/home/gvezzani/Desktop/PhD/Anno_3/multiple-superq/data-from-robot-for-testing-1809/point-cloud-grasps/domino_box_"$i --object domino_box --number $i --visualize_hand
done

END=5
for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --get_grasping_pose --hand both --remove-outliers "(0.01 10)" --file "/home/gvezzani/Desktop/PhD/Anno_3/multiple-superq/data-from-robot-for-testing-1809/point-cloud-grasps/domino_box_hor_"$i --object domino_box_hor --number $i --visualize_hand
done

END=6
for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --get_grasping_pose --hand both --remove-outliers "(0.01 10)" --file "/home/gvezzani/Desktop/PhD/Anno_3/multiple-superq/data-from-robot-for-testing-1809/point-cloud-grasps/jello_box_"$i --object jello_box --number $i --visualize_hand
done

END=4
for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --get_grasping_pose --hand both --remove-outliers "(0.01 10)" --file "/home/gvezzani/Desktop/PhD/Anno_3/multiple-superq/data-from-robot-for-testing-1809/point-cloud-grasps/pink_toy_"$i --object pink_toy --number $i --visualize_hand
done

END=5
for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --get_grasping_pose --hand both --remove-outliers "(0.01 10)" --file "/home/gvezzani/Desktop/PhD/Anno_3/multiple-superq/data-from-robot-for-testing-1809/point-cloud-grasps/red_spray_"$i --object red_spray --number $i --visualize_hand
done

END=3
for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --get_grasping_pose --hand both --remove-outliers "(0.01 10)" --file "/home/gvezzani/Desktop/PhD/Anno_3/multiple-superq/data-from-robot-for-testing-1809/point-cloud-grasps/tiger_"$i --object tiger --number $i --visualize_hand
done



