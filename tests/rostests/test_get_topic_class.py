#!/usr/bin/env python
#
##############################################################################
# Imports
##############################################################################

import unittest

import rospy
import rostest
import rostopic
import std_msgs.msg as std_msgs


##############################################################################
# Tests
##############################################################################

class TestTopicType(unittest.TestCase):
    def test_foo(self):
        topic = '/foo/r'
        msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=True)

        self.assertEqual(msg_class, std_msgs.ColorRGBA)
        self.assertEqual(real_topic, '/foo')

        msg = std_msgs.ColorRGBA(r=1.0)
        self.assertEqual(msg_eval(msg), 1.0)


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('test_get_topic_class')
    rostest.rosrun('ros_topics_watcher', 'test_get_topic_class', TestTopicType)
