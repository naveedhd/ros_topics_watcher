#!/usr/bin/env python
#
##############################################################################
# Imports
##############################################################################

import argparse
import rospy
import rostopic
import sys

from cPickle import dumps

from . import console


##############################################################################
# Argument Parsing
##############################################################################

def description():
    examples = ["--list-published-topics", "access_point odom/pose/pose/position"]
    script_name = "ros-topics-watcher"

    banner_line = console.green + "*" * 79 + "\n" + console.reset
    s = "\n"
    s += banner_line
    s += console.bold_white + \
        "ROS Topics Watcher".center(79) + "\n" + console.reset
    s += banner_line
    s += "\n"
    s += console.bold + "Examples" + console.reset + "\n\n"
    s += '\n'.join(["    $ " + console.cyan + script_name + console.yellow +
                    " {0}".format(example_args) + console.reset for example_args in examples])
    s += "\n\n"
    s += banner_line
    return s


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-l', '--list-published-topics', action='store_true',
                        default=False, help='Prints the list of published topics')
    parser.add_argument('topics', nargs=argparse.REMAINDER, default=None,
                        help='space separated list of topics to watch')
    return parser


##############################################################################
# Helpers
##############################################################################

class CachedSubscriber(object):
    def __init__(self):
        self.cached_pickle = {}

    def _is_changed(self, msg):
        current_pickle = dumps(msg, -1)
        if current_pickle == self.cached_pickle:
            return False

        self.cached_pickle = current_pickle
        return True

    def callback(self, msg):
        if self._is_changed(msg):
            print msg


def handle_args(args):
    if args.list_published_topics:
        rostopic._rostopic_list(None, verbose=True, publishers_only=True)
        return

    if args.topics:
        topic = args.topics[0]
        print topic

        rospy.init_node('ros_topics_watcher', anonymous=True)

        cached_sub = CachedSubscriber()
        msg_class, real_topic, _ = rostopic.get_topic_class(topic)
        sub = rospy.Subscriber(real_topic, msg_class, cached_sub.callback)

        while not rospy.is_shutdown():
            rospy.spin()


##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the topics watcher script.
    """
    command_line_args = rospy.myargv(argv=sys.argv)[1:]
    parser = command_line_argument_parser()
    args = parser.parse_args(command_line_args)
    handle_args(args)
