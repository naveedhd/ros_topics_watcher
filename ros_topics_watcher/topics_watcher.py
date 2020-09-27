#!/usr/bin/env python
#
##############################################################################
# Imports
##############################################################################

import argparse
import sys
import time

import rosgraph
import roslib
import rospy
import rostopic

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

class CallbackDiffEcho(rostopic.CallbackEcho):
    def __init__(self, topic):
        super(CallbackDiffEcho, self).__init__(topic, None)
        self.last_data = None
        self.filter_fn = self.filter_func

    def filter_func(self, data):
        eval_data = self.msg_eval(data) if self.msg_eval is not None else data
        if eval_data == self.last_data:
            return False

        self.last_data = eval_data
        return True

    def custom_strify_message(self, val,
                              indent='',
                              time_offset=None,
                              current_time=None,
                              field_filter=None,
                              type_information=None,
                              fixed_numeric_width=None,
                              value_transform=None):
        return self.topic + ': ' + \
                 super(CallbackDiffEcho, self).custom_strify_message(val,
                                                                     indent,
                                                                     time_offset,
                                                                     current_time,
                                                                     field_filter,
                                                                     type_information,
                                                                     fixed_numeric_width,
                                                                     value_transform)


def handle_args(args):
    if args.list_published_topics:
        rostopic._rostopic_list(None, verbose=True, publishers_only=True)
        return

    if not args.topics:
        return

    rostopic._check_master()
    node_name = 'ros_topics_watcher'
    rospy.init_node(node_name, anonymous=True)

    for topic in args.topics:
        # resolves namespace .. in this case making it global
        topic = rosgraph.names.script_resolve_name(node_name, topic)

        msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=True)
        if msg_class is None:
            # occurs on ctrl-C
            continue

        callback_echo = CallbackDiffEcho(topic)
        callback_echo.msg_eval = msg_eval

        # extract type information for submessages
        type_information = None
        if len(topic) > len(real_topic):
            subtopic = topic[len(real_topic):]
            subtopic = subtopic.strip('/')
            if subtopic:
                fields = subtopic.split('/')
                submsg_class = msg_class
                while fields:
                    field = fields[0].split('[')[0]
                    del fields[0]
                    index = submsg_class.__slots__.index(field)
                    type_information = submsg_class._slot_types[index]
                    if fields:
                        submsg_class = roslib.message.get_message_class(type_information.split('[', 1)[0])
                        if not submsg_class:
                            raise rostopic.ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % type_information)

        use_sim_time = rospy.get_param('/use_sim_time', False)
        sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, {'topic': topic, 'type_information': type_information})

        # TODO: this will block for next topics.. solve this
        if use_sim_time:
            # #2950: print warning if nothing received for two seconds

            timeout_t = time.time() + 2.
            while time.time() < timeout_t and \
                    callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                rostopic._sleep(0.1)

            if callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

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
