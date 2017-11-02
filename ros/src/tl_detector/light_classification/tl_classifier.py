from styx_msgs.msg import TrafficLight

import numpy as np
import os

import tensorflow as tf
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__)) # os.getcwd() doesn't work here since this file is called in tl_detector.py and current working dir would be tl_detector.py's dir

def translate_class(light_class):
    if light_class == 1:
        return TrafficLight.RED
    if light_class == 2:
        return TrafficLight.YELLOW
    if light_class == 3:
        return TrafficLight.GREEN
    return TrafficLight.UNKNOWN

def interpret_light(scores, classes):
    if scores[0][0] > 0.6:
       return translate_class(classes[0][0])
    else:
        return TrafficLight.UNKNOWN

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        ## What model to download.
        MODEL_NAME = os.path.join(CURRENT_DIR, 'ssd_mobilenet_traffic_lights')

        ## Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
        ##NUM_CLASSES = 3

        ## Load a (frozen) Tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        with self.detection_graph.as_default():
          with tf.Session(graph=self.detection_graph) as sess:
            
            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            ## detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            ## num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_state = TrafficLight.UNKNOWN

        with self.detection_graph.as_default():
          with tf.Session(graph=self.detection_graph) as sess:
            
            # Definite input and output Tensors for detection_graph
            ##image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            ##detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            ##detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            ##detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            ##num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
            
            image_np = image #load_image_into_numpy_array(image)
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)
            # Actual detection.
            (scores, classes) = sess.run(
                [self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: image_np_expanded})

            light_state = interpret_light(scores, classes)
        
        #TODO implement light color prediction
        return light_state
