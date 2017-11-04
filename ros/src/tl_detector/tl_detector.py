#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

import math

STATE_COUNT_THRESHOLD = 3
temp = False

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        if temp is True:
            self.temp_pub = rospy.Publisher('/image_sample', Image, queue_size=1)
            self.last_pub_time = None

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.site_instead_of_sim = rospy.get_param('~site_instead_of_sim', False)
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.loginfo('time now: %s', rospy.Time.now().to_sec())

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights() # <---------------------- computationally slow (1)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #rospy.loginfo('time now: %s', rospy.Time.now().to_sec())
        if temp is True:
            if self.last_pub_time is None:
                time_since_last_pub = 100
            else:
                time_since_last_pub = rospy.Time.now().to_sec() - self.last_pub_time.to_sec()
            #rospy.loginfo('msg.header.stamp.to_sec(): %s', msg.header.stamp.to_sec())
            #rospy.loginfo('time_since_last_pub: %s', time_since_last_pub)
            if time_since_last_pub > 1.5:
                self.temp_pub.publish(msg)
                self.last_pub_time = rospy.Time.now()

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        nearest = 1e6
        nearest_waypoint  = 0

        ## loop through all base waypoints to compare may not be a good idea
        range_lower = 0
        range_upper = len(self.waypoints.waypoints)

        for i in range(range_lower, range_upper):
            distance = self.dist(self.waypoints.waypoints[i].pose.pose.position, pose.position)
            if distance is not None and distance < nearest:
                nearest = distance
                nearest_waypoint  = i

        return nearest_waypoint

    def dist(self,pos1,pos2):
        if pos1 is None or pos2 is None:
            return None
        else:
            return math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2 + (pos1.z-pos2.z)**2)

    def get_closest_vis_light(self, car_position, stop_line_positions):
        ## car_position: nearest waypoint to the ego car

        if car_position is None:
            return None, -1

        nearest = 1e6
        nearest_light_index = 0
        visible_distance = 170

        for i in range(len(stop_line_positions)):
            car_x = self.waypoints.waypoints[car_position].pose.pose.position.x
            car_y = self.waypoints.waypoints[car_position].pose.pose.position.y
            distance = math.sqrt( (car_x-stop_line_positions[i][0])**2 + (car_y-stop_line_positions[i][1])**2 )
            if distance is not None and distance < nearest:
                nearest = distance
                nearest_light_index  = i
        ##rospy.loginfo('nearest stopline: %s', nearest)

        if nearest < visible_distance:
            light = self.lights[nearest_light_index]
            stop_line_pose = Pose()
            stop_line_pose.position.x = stop_line_positions[nearest_light_index][0]
            stop_line_pose.position.y = stop_line_positions[nearest_light_index][1]
            stop_line_pose.position.z = 0
            light_wp = self.get_closest_waypoint(stop_line_pose)
            if car_position <= light_wp or self.site_instead_of_sim: # if it's actual test trac (site), then condition "car_position <= light_wp" doesn't apply
                return light, light_wp

        return None, -1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8") #originally bgr8

        if temp is True:
            ## temp code start ---
            return light.state
            ## temp code end ---

        #Get classification
        light_state = self.light_classifier.get_classification(cv_image) # <---------------------- computationally slow (1.1.1)
        rospy.loginfo('light_state: %s', light_state)
        return light_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.waypoints is None:
            return -1, TrafficLight.UNKNOWN

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        car_position = None
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        light, light_wp = self.get_closest_vis_light(car_position,stop_line_positions)
        ##rospy.loginfo("light:  %s", light)
        if light:
            state = self.get_light_state(light) # <---------------------- computationally slow (1.1)
            return light_wp, state
        ##self.waypoints = None

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
