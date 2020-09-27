#!/usr/bin/env python
#
##############################################################################
# Imports
##############################################################################

import argparse
import socket
import sys

import rospy
import rostopic
import rosgraph

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

    if args.topics:
        # TODO: take all topics
        topic = args.topics[0]

        # resolves namespace .. in this case making it global
        topic = rosgraph.names.script_resolve_name('ros-topics-watcher', topic)

        try:
            rostopic._rostopic_echo(topic, CallbackDiffEcho(topic))
        except socket.error:
            sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")


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
