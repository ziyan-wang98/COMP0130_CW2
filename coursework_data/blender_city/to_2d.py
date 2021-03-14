#!/usr/bin/env python3

import os
import mathutils
import numpy as np
import math

# input
poses_path = "/home/martin/Documents/UCL/TAing/Robotics/compgx04/coursework_2/blender/camera_poses.txt"
observations_path = "/home/martin/Documents/UCL/TAing/Robotics/compgx04/coursework_2/blender/tracks_dist.txt"
landmarks_path = "/home/martin/Documents/UCL/TAing/Robotics/compgx04/coursework_2/blender/landmarks_3d.txt"

# output
measurements_out_path = "/home/martin/Documents/UCL/TAing/Robotics/compgx04/coursework_2/blender/measurements_2d.txt"
poses_out_path = "/home/martin/Documents/UCL/TAing/Robotics/compgx04/coursework_2/blender/poses_2d.txt"

# stats
min_marks = 9999

marks = np.loadtxt(landmarks_path)
current_frame = 1


with \
        open(poses_path, "r") as cam_file,\
        open(observations_path, "r") as observation_file,\
        open(measurements_out_path, "w") as measurements_out_file,\
        open(poses_out_path, "w") as poses_out_file:

    for cam_line, observation_line in zip(cam_file, observation_file):
        cam = [float(s) for s in cam_line.split()]
        cam_pos = np.array(cam[1:3])

        # Compute 2D camera location (x,y,bearing[-pi..+pi])
        bearing = math.atan2(cam_pos[1], cam_pos[0])

        observation = observation_line.split()
        landmark_ids = [int(s) for s in observation[0::4]]

        measurement = []
        exported_marks = 0
        for lid in landmark_ids:
            if lid % 2 == 0:  # Reduce number of landmarks
                continue
            mark_pos = marks[lid,0:2]
            d = np.linalg.norm(mark_pos-cam_pos)
            angle_diff = math.atan2(mark_pos[1], mark_pos[0]) - bearing
            r_bearing = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            exported_marks += 1
            measurements_out_file.write("{} {} {} {} {} ".format(lid, mark_pos[0], mark_pos[1], d, bearing))

        if exported_marks < min_marks:
            min_marks = exported_marks
        measurements_out_file.write("\n")

        # Write output
        poses_out_file.write("{} {} {} {}\n".format(current_frame, cam_pos[0], cam_pos[1], bearing))

        current_frame += 1

print("Done, frames have at least", min_marks, "landmarks")



