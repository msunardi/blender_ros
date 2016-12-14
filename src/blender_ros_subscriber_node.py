#!/usr/bin/python

# REF: https://gist.github.com/jobliz/2596594
import rospy
from std_msgs.msg import Float32, Float64, String, Bool
from arbotix_msgs.srv import *
from ast import literal_eval as le

import redis
import threading

class BlenderSubscriber(threading.Thread):
    def __init__(self, r, channels):
        threading.Thread.__init__(self)

        self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=10)        
        self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=10)
        self.right_sho_pitch = rospy.Publisher('/right_sho_pitch/command', Float64, queue_size=10)
        self.right_sho_roll = rospy.Publisher('/right_sho_roll/command', Float64, queue_size=10)
        self.right_elbow = rospy.Publisher('/right_elbow/command', Float64, queue_size=10)
        self.left_sho_pitch = rospy.Publisher('/left_sho_pitch/command', Float64, queue_size=10)
        self.left_sho_roll = rospy.Publisher('/left_sho_roll/command', Float64, queue_size=10)
        self.left_elbow = rospy.Publisher('/left_elbow/command', Float64, queue_size=10)
        
        self.redis = r
        self.pubsub = self.redis.pubsub()
        self.pubsub.subscribe(channels)
        self.rate = rospy.Rate(30)

        self.mapping = {
            'Head': {'z': None,
                     'x': self.head_tilt_pub,
                     'y': None #self.head_pan_pub
            },
            'Neck': {'z': None,
                     'x': None, #self.head_tilt_pub,   
                     'y': self.head_pan_pub
            },
            'RElbow': {'z': None,
                       'x': self.right_elbow,
                       'y': None
            },
            'LElbow': {'z': None,
                       'x': self.left_elbow,
                       'y': None
            },
            'RShoulder': {'z': self.right_sho_pitch,
                          'x': self.right_sho_roll,
                          'y': None
            },
            'LShoulder': {'z': self.left_sho_pitch,
                          'x': self.left_sho_roll,
                          'y': None
            }
        }

    def work(self, item):
        print( "%s: %s" % (item['channel'], item['data']))
        if item['channel'] != 'test':
            return
        data = le(str(item['data']))
        print "Data: %s" % data
        # if type(data) == dict:
        #     if 'Head' in data.keys():
        #         head_pan = data['Head']['y']
        #         head_tilt = data['Head']['x']
        #         self.head_pan_pub.publish(Float64(head_pan))
        #         self.head_tilt_pub.publish(Float64(head_tilt))
        #     if 'Neck' in data.keys():
        #         head_pan = data['Neck']['y']
        #         head_tilt = data['Neck']['x']
        #         self.head_pan_pub.publish(Float64(head_pan))
        #         self.head_tilt_pub.publish(Float64(head_tilt))
        #     if 'RShoulder' in data.keys():
        #         pitch = data['RShoulder']['z']
        #         roll = data['RShoulder']['x']
        #         self.mapping['RShoulder']['x'].publish(Float64(pitch))
        #         self.mapping['RShoulder']['z'].publish(Float64(roll))

        # if type(data) == dict and 'RShoulder' in data.keys():
        #     pitch = data['RShoulder']['z']
        #     roll = data['RShoulder']['x']
        #     self.mapping['RShoulder']['x'].publish(Float64(pitch))
        #     self.mapping['RShoulder']['z'].publish(Float64(roll))
        if type(data) == dict:
            for joint in data:
                if joint not in self.mapping.keys():
                    return
                pose = data[joint]
                for axis in ['z','x','y']:
                    pub = self.mapping[joint][axis]
                    theta = pose[axis]

                    print "Publisher: %s, theta: %s" % (pub, theta)
                    if pub:
                        pub.publish(Float64(theta))

    def run(self):
        rospy.loginfo("[BlenderSubscriber]: Listening to Blender stream from Redis ...")

        while not rospy.is_shutdown():
            for item in self.pubsub.listen():
                if item['data'] == "KILL":
                    self.pubsub.unsubscribe()
                    print(self, "Unsubscribed and finished")
                    break
                else:
                    self.work(item)

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('blender_ros_subscriber_node', anonymous=True)

    r = redis.Redis()
    client = BlenderSubscriber(r, ['test', 'test2'])
    client.start()
    rospy.spin()