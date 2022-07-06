## ignore this file


# import joblib
# from sensor_msgs.msg import LaserScan, CameraInfo
# import numpy as np
# # %matplotlib qt
# # import cv2
# import math
# from matplotlib import pyplot as plt

# # https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/
# # assuming a horizontal camera viewing angle of 62.2 degrees in model.sdf <horizontal_fov>1.085595</horizontal_fov> == 62,2 degrees
# # the sensor ranges start straight to the front. to get all ranges to the left msg.ranges[:31] and to the right msg.ranges[-31:] 


# msg = joblib.load('/home/parallels/stored_joblib/scan_msg.joblib')
# cv_frame = joblib.load('/home/parallels/stored_joblib/cv_frame.joblib')
# result_xywhn = joblib.load('/home/parallels/stored_joblib/result_xywhn.joblib')
# # plt.imshow(cv_frame)
# # plt.show()
# # cv2.imshow('l' , np.array(cv_frame, dtype = np.uint8 ))
# # cv2.waitKey()
# # nr_bb = result_xywhn.shape[0]

# np_xywhn = np.array(result_xywhn)

# sensor_ranges_lr = np.append(msg.ranges[-31:], msg.ranges[:31])

# angel_increment = msg.angle_increment
# laser_pixel_intervall = 640 / (1.085595 / angel_increment)

# x1 = np_xywhn[:, 0] * 640
# x2 = (np_xywhn[:, 0] + np_xywhn[:, 2]) * 640
# labels = np_xywhn[:, 5]
# ranges = []
# angles = []
# points_x = []
# points_y = []

# for i in range(len(np_xywhn)):
#   laser_index_1 = int(x1[i]/laser_pixel_intervall)
#   laser_index_2 = int(x2[i]/laser_pixel_intervall) + 1
#   range = np.min(sensor_ranges_lr[laser_index_1:laser_index_2])
#   ranges.append(range)
#   # angle = (laser_index_1 * angel_increment) -1.5708
#   if laser_index_1 < 31: 
#     angle = 7.85398 - ((laser_index_1 + 329) * angel_increment)
#   else:
#     angle = 1.5708 - ((laser_index_1 -31) * angel_increment)
#   points_x.append(range*(math.cos((angle))))
#   points_y.append(range*(math.sin((angle))))


# # plt.xlim(0, 5)
# # plt.ylim(0, 5)
# # plt.grid()
# # # plt.plot(points_x, points_y)
# # plt.plot([1], [3], 'ro')
# # plt.show()

# plt.rcParams["figure.figsize"] = [7.00, 3.50]
# plt.rcParams["figure.autolayout"] = True
# x = points_x
# y = points_y
# plt.xlim(-5, 5)
# plt.ylim(-5, 5)
# plt.grid()
# plt.plot(x, y, marker="o", markersize=20, markeredgecolor="red", markerfacecolor="green")
# plt.show()


############################################################################################### 
# in the scan msg: set the values to inf that dont correspond to a cone
import joblib
import numpy as np

msg = joblib.load('/home/parallels/stored_joblib/scan_msg.joblib')
# result_xywhn = joblib.load('/home/parallels/stored_joblib/result_xywhn.joblib')
result_xywhn = joblib.load('/home/parallels/stored_joblib/bba.joblib')

result_xywhn  = np.array(result_xywhn.data).reshape(len(result_xywhn.data)//6, 6)
# write a node, that takes the scan msg and the result_xywhn and replaces the inf values with the min value of the sensor ranges



sensor_ranges_lr = np.append(msg.ranges[-31:], msg.ranges[:31])

angel_increment = msg.angle_increment
laser_pixel_intervall = 640 / (1.085595 / angel_increment)

x1 = result_xywhn[:, 0] * 640
x2 = (result_xywhn[:, 0] + result_xywhn[:, 2]) * 640
cone_range_index = []

for i in range(len(result_xywhn)):
  laser_index_1 = int(x1[i]/laser_pixel_intervall) - 5
  laser_index_2 = int(x2[i]/laser_pixel_intervall) + 2
  start_index = np.where(sensor_ranges_lr[laser_index_1:laser_index_2] != np.inf)
  cone_index = start_index[0] + laser_index_1
  for index in cone_index:
    if index < 31: 
      cone_range_index.append(index + 329)
    else:
      cone_range_index.append(index - 31)

l = [ind for ind in range(360) if ind not in cone_range_index]
for b in l:
  msg.ranges[b] = np.inf

# publish msg


# # %matplotlib qt
# from matplotlib import pyplot as plt


# plt.rcParams["figure.figsize"] = [7.00, 3.50]
# plt.rcParams["figure.autolayout"] = True
# x = points_x
# y = points_y
# plt.xlim(-5, 5)
# plt.ylim(-5, 5)
# plt.grid()
# plt.plot(x, y, marker="o", markersize=20, markeredgecolor="red", markerfacecolor="green")
# plt.show()
