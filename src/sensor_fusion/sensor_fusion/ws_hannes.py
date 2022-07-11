# dev imports
import joblib
from matplotlib import pyplot as plt
import os
import matplotlib
%matplotlib qt


# required imports
import numpy as np
import math
import pandas as pd
from transforms3d import euler


# extracted functions
def filter_for_time(msg, scan_msgs, odom_msgs):
  image_time_sec, image_time_nanosec = msg.data[-2], msg.data[-1]
  image_time = image_time_sec + image_time_nanosec / 1e9

  # match scan msg to image_time
  time_delta_scan = []
  for m in scan_msgs:
    scan_time = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
    time_delta_scan.append(np.abs(image_time - scan_time))
  scan_msg = scan_msgs[np.where(np.array(time_delta_scan) == np.min(time_delta_scan))[0][0]]

  # match scan msg to image_time
  time_delte_odom = []
  for m in odom_msgs:
    odom_time = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
    time_delte_odom.append(np.abs(image_time - odom_time))
  odom_msg = odom_msgs[np.where(np.array(time_delte_odom) == np.min(time_delte_odom))[0][0]]

  # TODO emty the arrays according to the time delta

  # pop the time info from the bounding boxes and convert to numpy array
  msg.data = msg.data[:-2]
  bboxes  = np.array(msg.data).reshape(len(msg.data)//6, 6)
  return scan_msg, odom_msg, bboxes

# class variables
scan_msgs = None
odom_msgs = None
starting_pose = None
starting_x = None
starting_y = None
cone_positions = None
point_confidence_intervall = 0.25
cone_positions = pd.DataFrame(columns=['x', 'y', 'label'])


##### iteration over msgs bundles to simulate arriaval of new data
stored_data_path = '/home/parallels/stored_joblib/'
list_of_data_files = os.listdir(stored_data_path)

##### content of def subscribe_bounding_boxes
for a, values in enumerate(list_of_data_files[:1]):
  msgs = joblib.load(os.path.join(stored_data_path, 'msg_dump{}.joblib'.format(a)))

  scan_msgs = msgs['scan_msgs']
  odom_msgs = msgs['odom_msgs']
  msg = msgs['bounding_boxes']

  # 1. match the messages on a time basis
  scan_msg, odom_msg, bboxes = filter_for_time(msg, scan_msgs, odom_msgs)

  # 2. get orientation delta of the bot
  # def get_pose(odom_msg):
  pose_quaternion = [odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z]
  current_pose = euler.quat2euler(pose_quaternion, axes='rxyz')
  if starting_pose == None:
    starting_pose = current_pose
    starting_x = odom_msg.pose.pose.position.x
    starting_y = odom_msg.pose.pose.position.y
  pose_delta = current_pose[1] - starting_pose[1]
  x_delta = - odom_msg.pose.pose.position.x - starting_x 
  y_delta = - odom_msg.pose.pose.position.y - starting_y

  # orientations

  # 3. get relative positions of cones and turn them into absolut coordinates
  # def get_cone_coordinats(bboxes, scan_msg):

  relevant_sensor_ranges = np.append(scan_msg.ranges[-31:], scan_msg.ranges[:31])

  angel_increment = scan_msg.angle_increment
  laser_pixel_intervall = 640 / (1.085595 / angel_increment)

  x1 = (bboxes[:, 0] - (bboxes[:, 2] / 2)) * 640
  x2 = (bboxes[:, 0] + (bboxes[:, 2] / 2)) * 640

  labels = bboxes[:, 5]

  # get range and x y coordinates of the cones (bounding boxes, laser scan, orientation of the robot in the world) 
  for b, value in enumerate(bboxes):
    laser_index_1 = int(x1[b]/laser_pixel_intervall)
    laser_index_2 = int(x2[b]/laser_pixel_intervall) + 1
    range = np.min(relevant_sensor_ranges[laser_index_1:laser_index_2])
    start_index = np.where(relevant_sensor_ranges == range)[0][0]
    if range != np.inf:
      if start_index <= 31:
        angle = (7.85398 - ((start_index + 329) * angel_increment)) #+ pose_delta
      else:
        angle = (1.5708 - ((start_index -31) * angel_increment)) #+ pose_delta
      x_coordinate =  range*(math.cos((angle))) + x_delta
      y_coordinate = range*(math.sin((angle))) + y_delta
      # check if cone is alreasy in the DataFrame
      if len(cone_positions) > 0:
        existing_cone = cone_positions.loc[
          ((x_coordinate - point_confidence_intervall) < cone_positions['x']) & 
          ((x_coordinate + point_confidence_intervall) > cone_positions['x']) &
          ((y_coordinate - point_confidence_intervall) < cone_positions['y']) &
          ((y_coordinate + point_confidence_intervall) > cone_positions['y']) 
          ]
        if len(existing_cone) == 0:
          cone_positions.loc[len(cone_positions)] = [x_coordinate, y_coordinate, labels[b]]
      else:
        cone_positions.loc[len(cone_positions)] = [x_coordinate, y_coordinate, labels[b]]
  

plt.scatter(cone_positions.loc[cone_positions["label"] == 0]['x'], cone_positions.loc[cone_positions["label"] == 0]['y'], c='blue', s=200)
plt.scatter(cone_positions.loc[cone_positions["label"] == 1]['x'], cone_positions.loc[cone_positions["label"] == 1]['y'], c='orange', s=200)
plt.scatter(cone_positions.loc[cone_positions["label"] == 2]['x'], cone_positions.loc[cone_positions["label"] == 2]['y'], c='yellow', s=200)
plt.grid()
plt.xlim(-5, 5)
plt.ylim(-5, 5)
plt.show()
