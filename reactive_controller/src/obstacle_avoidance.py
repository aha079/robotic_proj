#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import tf
import copy

class ObstacleAvoiance:
  def __init__(self):
    rospy.init_node('reading_laser', anonymous=True)
    # self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.vel_pub = rospy.Publisher('/new_vel', Twist, queue_size=1)
    self.scan_sub= rospy.Subscriber("/scan", LaserScan , self.callback_scan)
    self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
    self.x = None
    self.y = None
    self.yaw = None
    self.ranges = None
    self.angle_min = None
    self.angle_max = None
    self.angle_increment = None
    self.time_increment = None
    self.scan_time = None
    self.range_min = None
    self.range_max = None

  def callback_odometry(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y

    yaw, pitch , roll = self.quaternion_to_euler(msg)
    self.yaw = yaw

  def quaternion_to_euler(self, msg):
    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    return (yaw, pitch , roll)


  def callback_scan(self, msg):
    self.ranges = list(msg.ranges)
    self.angle_min = msg.angle_min
    self.angle_max = msg.angle_max
    self.angle_increment = msg.angle_increment
    self.time_increment = msg.time_increment
    self.scan_time = msg.scan_time
    self.range_min = msg.range_min
    self.range_max = msg.range_max

  def pair_making(self, ranges):
    pairs = []
    for i in range(len(ranges)):
        if ranges[i] == math.inf:
            ranges[i] = self.range_max
        pairs.append((i, round(ranges[i], 3)))
    return pairs


  def smoothing(self, ranges):
    window_size = 5
    size = len(ranges) - 1
    
    for i in range(0, len(ranges)):
        if i == 0 or i == 1:
            ranges[i] = (2*ranges[i] + 6*ranges[i + 1] + 2*ranges[i + 2]) / 10
        elif i == 2:
            ranges[i] = (2*ranges[i - 1] + 6*ranges[i] + 1*ranges[i + 1] + 1*ranges[i + 2]) / 10
        elif i == size - 2:
            ranges[i] = (1*ranges[i - 2] + 1*ranges[i - 1] + 6*ranges[i] + 2*ranges[i + 1]) / 10
        elif i == size - 1 or i == size:
            ranges[i] = (1*ranges[i - 2] + 3*ranges[i - 1] + 6*ranges[i]) / 10

        else:
            ranges[i] = (1*ranges[i-2] + 3*ranges[i-1] + 6*ranges[i] + 3*ranges[i+1] + 1*ranges[i+2]) / 14
            
    return ranges

  def make_slice_indices(self, pairs, threshold, mode='gap'):
    if mode == 'gap':
      filtered_data = [x for x in pairs if x[1] >= threshold]
      slice_indices = []
      for index, pair in enumerate(filtered_data):
          try:
              angle = pair[0]
              if filtered_data[index + 1][0] != (angle + 1):
                  slice_indices.append(index + 1)
          except: 
              pass

      return slice_indices, filtered_data
    else:
      filtered_data = [x for x in pairs if x[1] <= threshold]
      slice_indices = []
      for index, pair in enumerate(filtered_data):
          try:
              angle = pair[0]
              if filtered_data[index + 1][0] != (angle + 1):
                  slice_indices.append(index + 1)
          except: 
              pass

      return slice_indices, filtered_data

  def slicing_gaps(self, slice_indices, filtered_data):
    final_gaps = []
    start = 0
    for index in slice_indices:
        end = index
        final_gaps.append(filtered_data[start: end])
        start = end
        
    final_gaps.append(filtered_data[start:])
    return final_gaps

  def main_VFH(self):
      rate = rospy.Rate(10) # 10hz
      ang_K_p = 1.8
      threshold = 3

      while not rospy.is_shutdown():
        twist = Twist()

        if not self.ranges or not self.x or not self.y:
          continue

        # ranges = self.smoothing(copy.copy(self.ranges))
        slice_indices, filtered_data = self.make_slice_indices(self.pair_making(self.ranges), threshold)
        gaps = self.slicing_gaps(slice_indices, filtered_data)


        final_list = []
        for gap in gaps:   
          weights = [np.interp(gap[i][1], [threshold, self.range_max], [1, 10]) for i in range(len(gap))]
          avg_angle = sum([gap[i][0] * weights[i] for i in range(len(gap))]) / sum(weights)

          final_list.append({
              "gap_list": gap,
              "avg_angle": avg_angle,
              "diff": avg_angle if avg_angle <= 180 else -(360 - avg_angle),
          })
            
            
        width_limit = 15
        final_list.sort(key=lambda item: abs(item['diff']))

        widest_list = []
        for l in final_list:
          if abs(l['diff']) < 100 and len(l['gap_list']) >= width_limit:
            widest_list.append(copy.deepcopy(l))
          
        widest_list.sort(key=lambda item: len(item['gap_list']), reverse=True)

        # final_list.sort(key=lambda item: len(item['gap_list']))
       if len(final_list[0]['gap_list']) < width_limit and widest_list:
          print("widest")
          diff = widest_list[0]['diff']
        else: 
          print("theta")
          diff = final_list[0]['diff']

        print('THETA')
        for l in final_list:
          print(f'    avg: {l["avg_angle"]},  diff: {l["diff"]} , len: {len(l["gap_list"])}')

        print('WIDEST')
        for l in widest_list:
          print(f'    avg: {l["avg_angle"]},  diff: {l["diff"]} , len: {len(l["gap_list"])}')
        
        print(f'selected diff: {diff}')
        print()


        linear = 0.8
        ang_P = 1.6 * diff * np.pi / 180 # degree to radian

        twist.angular.z = ang_P
        twist.linear.x = linear

        self.vel_pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
  try:
    obs = ObstacleAvoiance()
    obs.main_VFH()
  except rospy.ROSInterruptException:
    pass
