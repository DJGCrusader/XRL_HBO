#!/usr/bin/env python
# IMU to joint_states publisher
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header

IMUState = [0,0,0, 0,0,0, 0,0,0, 0,0,0]

def IMUParser():
    pub = rospy.Publisher('joint_states', JointState, queue_size=50)
    rospy.init_node('IMUParser')
    rate = rospy.Rate(50) # 10hz

    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['helmet_yaw', 'helmet_pitch', 'helmet_roll', 'chest_yaw', 'chest_pitch', 'chest_roll',
                      'right_shoulder_yaw', 'right_shoulder_pitch', 'right_shoulder_roll',
                      'right_elbow']
    subTo = ['zelmet/imu/rpy','/suit/chest/imu/rpy', '/suit/upper_right_arm/imu/rpy', '/suit/lower_right_arm/imu/rpy']
    rospy.Subscriber(subTo[0], Vector3Stamped, IMUZelmetCallback)
    rospy.Subscriber(subTo[1], Vector3Stamped, IMUChestCallback)
    rospy.Subscriber(subTo[2], Vector3Stamped, IMURUpperArmCallback)
    rospy.Subscriber(subTo[3], Vector3Stamped, IMURLowerArmCallback)

    hello_str.position = [IMUState[2], IMUState[1], IMUState[0], IMUState[5], IMUState[4],
                             IMUState[3], IMUState[8], IMUState[7], IMUState[6], IMUState[9]]
    hello_str.velocity = []
    hello_str.effort = []
    while not rospy.is_shutdown():
        hello_str = JointState()
        hello_str.header = Header()
        hello_str.header.stamp = rospy.Time.now()
        hello_str.name = ['helmet_yaw', 'helmet_pitch', 'helmet_roll', 'chest_yaw', 'chest_pitch', 'chest_roll',
                          'right_shoulder_yaw', 'right_shoulder_pitch', 'right_shoulder_roll',
                          'right_elbow']
        hello_str.position = [IMUState[2] - IMUState[5], IMUState[1] - IMUState[4], IMUState[0] - IMUState[3],
                                 IMUState[5], IMUState[4], IMUState[3],
                                  IMUState[8]- IMUState[5], IMUState[7]- IMUState[4], IMUState[6]- IMUState[3],
                                   IMUState[9]]
        hello_str.velocity = []
        hello_str.effort = []
        pub.publish(hello_str)
        # rate.sleep()

def IMUZelmetCallback(data):
    IMUState[0:3] = [data.vector.x, data.vector.y, data.vector.z]
def IMUChestCallback(data):
    IMUState[3:6] = [data.vector.x, data.vector.y, data.vector.z]
def IMURUpperArmCallback(data):
    IMUState[6:9] = [data.vector.x, data.vector.y, data.vector.z]
def IMURLowerArmCallback(data):
    IMUState[9:12] = [data.vector.x, data.vector.y, data.vector.z]

if __name__ == '__main__':
    try:
        IMUParser()
    except rospy.ROSInterruptException:
        pass
