#!/usr/bin/env python

import rospy
import sensor_msgs.msg

def navsatfix_callback(data):
    pub.publish(data)

def main():
    rospy.init_node('nmea_sentence_to_gps_fix', anonymous=True)
    rospy.Subscriber('/nmea_sentence', sensor_msgs.msg.NavSatFix, navsatfix_callback)
    pub = rospy.Publisher('gps/fix', sensor_msgs.msg.NavSatFix, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
