#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from std_msgs.msg import Int32

from tf.transformations import euler_from_quaternion 

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL     = 0.5 

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []
        self.final_waypoints = []
        self.stop_wp = -1 
        self.moving = False
        self.last_position = None
        self.last_pub_time = None
        self.last_nxt_wp   = None
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement

        dist_tol = 0.01
        time_tol = 0.05 ##0.1
        if self.last_pub_time is None:
            time_since_last_pub = 100
        else:
            time_since_last_pub = msg.header.stamp.to_sec() - self.last_pub_time.to_sec()

        #rospy.loginfo('time_since_last_pub: %s', time_since_last_pub)
        
        if self.last_position is None:
            self.moving = True
        elif self.dist(msg.pose.position, self.last_position) < dist_tol:
            self.moving = False
        else:
            self.moving = True
        #rospy.loginfo('self.moving: %s', self.moving)
        if len(self.base_waypoints) < 1 or time_since_last_pub < time_tol :
            ## only calc final_waypoints when certain conditions are met
            pass
        else:
            ## get the next waypoint (index)
            next_waypoint = self.get_next_waypoint(msg.pose)
            #rospy.loginfo('next_waypoint: %s', next_waypoint)

            ## update the final_waypoints
            self.final_waypoints = self.get_final_waypoints(next_waypoint)
            #rospy.loginfo('final_waypoints[1]: %s', self.final_waypoints[1])

            self.last_nxt_wp    = next_waypoint
            self.last_position  = msg.pose.position

        if len(self.final_waypoints) > 1 and time_since_last_pub >= time_tol :
            ## only publish the final_waypoints with header when /current_pose meassage received and certain time has passed
            self.publish()


    def publish(self):
        
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints    = self.final_waypoints

        self.final_waypoints_pub.publish(lane)

        self.last_pub_time = rospy.Time.now()

    def get_next_waypoint(self,pose):

        next_waypoint = self.get_nearest_waypoint(pose.position)
        if next_waypoint < len(self.base_waypoints)-1:
            orientation = self.base_waypoints[next_waypoint].pose.pose.orientation
            quatn = [orientation.x,orientation.y,orientation.z,orientation.w]
            (roll,pitch,yaw) = euler_from_quaternion(quatn)
            #rospy.loginfo('next_waypoint yaw: %s', yaw)
            wp_position = self.base_waypoints[next_waypoint].pose.pose.position
            dx = wp_position.x - pose.position.x
            dy = wp_position.y - pose.position.y 
            angle_car_to_nxt_wp = math.atan2(dy,dx)

            diff_angle = abs(angle_car_to_nxt_wp - yaw) # both angles are supposed to be within [0,pi] and [-pi,0]
            diff_angle = min(diff_angle, 2*math.pi - diff_angle)

            if diff_angle >= math.pi/2:
                next_waypoint += 1

        return next_waypoint 

    def get_nearest_waypoint(self,position):

        nearest = 1e6
        nearest_waypoint = self.last_nxt_wp

        ## loop through all base waypoints to compare may not be a good idea
        if self.last_nxt_wp is None: 
            range_lower = 0
            range_upper = len(self.base_waypoints)
        else:
            distance = self.dist(self.base_waypoints[self.last_nxt_wp].pose.pose.position,
                                                                                 position)
            extend_wp = max(2 * int(distance) + 2, 50)
            range_lower = max(0, (self.last_nxt_wp - extend_wp))
            range_upper = min(len(self.base_waypoints), (self.last_nxt_wp + 2 * extend_wp))

        for i in range(range_lower, range_upper):
            distance = self.dist(self.base_waypoints[i].pose.pose.position, position)
            if distance is not None and distance < nearest:
                nearest = distance
                nearest_waypoint  = i 

        return nearest_waypoint 

    def dist(self,pos1,pos2):
        if pos1 is None or pos2 is None:
            return None
        else:
            return math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2 + (pos1.z-pos2.z)**2)

    def get_final_waypoints(self, next_waypoint):

        final_waypoints = []
        for i in range(next_waypoint, min(next_waypoint+LOOKAHEAD_WPS,len(self.base_waypoints))):
            p = Waypoint()
            waypoint = self.base_waypoints[i]
            p.pose.pose.position.x    = waypoint.pose.pose.position.x
            p.pose.pose.position.y    = waypoint.pose.pose.position.y
            p.pose.pose.position.z    = waypoint.pose.pose.position.z
            p.pose.pose.orientation.x = waypoint.pose.pose.orientation.x
            p.pose.pose.orientation.y = waypoint.pose.pose.orientation.y
            p.pose.pose.orientation.z = waypoint.pose.pose.orientation.z
            p.pose.pose.orientation.w = waypoint.pose.pose.orientation.w
            linear_vel                = waypoint.twist.twist.linear.x
            ## modify linear velocity for this waypoint if there is an upcoming redlight 
            if self.stop_wp > 0: # i.e., not -1
                stop_margin = 2.5 ## meter 
                if i >= self.stop_wp:
                    vel = 0.
                else:
                    dist = self.distance(self.base_waypoints, i, self.stop_wp)
                    if dist < stop_margin:
                        vel =  0.
                    else: 
                        vel = math.sqrt(2 * MAX_DECEL * (dist - stop_margin ))
                if vel < 1.:
                    vel = 0.
                linear_vel = min(vel, linear_vel)
            p.twist.twist.linear.x    = linear_vel
            ## put this way point in the list 
            final_waypoints.append(p)

        return final_waypoints

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_wp = msg.data 
        ##rospy.loginfo('stop waypoint: ', self.stop_wp)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

