#!/usr/bin/env python
#
##############################################################################
# Imports
##############################################################################

import argparse
import operator
import re
import sys
import threading
import yaml

import rosgraph
import roslib
import rospy
import rostopic


##############################################################################
# Argument Parsing
##############################################################################

def description():
    examples = ["--list-published-topics", "access_point odom/pose/pose/position"]
    script_name = "ros-topics-watcher"

    banner_line = "*" * 79 + "\n"
    s = "\n"
    s += banner_line
    s += "ROS Topics Watcher".center(79) + "\n"
    s += banner_line
    s += "\n"
    s += "Examples" + "\n\n"
    s += '\n'.join(["    $ " + script_name +
                    " {0}".format(example_args) for example_args in examples])
    s += "\n\n"
    s += banner_line
    return s


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-l', '--list-published-topics', action='store_true',
                        default=False, help='Prints the list of published topics')
    parser.add_argument('topics', nargs='+', default=None,
                        help='space separated list of topics to watch')
    return parser


##############################################################################
# Helpers
##############################################################################

class CallbackDiffEcho(rostopic.CallbackEcho):
    def __init__(self, topic):
        super(CallbackDiffEcho, self).__init__(topic, None)
        self.last_data = None

        # TODO: capture original filter func too
        self.filter_fn = self.filter_func

    def filter_func(self, data):
        if self.msg_eval is not None:
            data = self.msg_eval(data)

        if data == self.last_data:
            return False

        self.last_data = data
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


def attr_eval(msg_eval, attrs):
    attr_getter = operator.attrgetter(*attrs)

    def func(data):
        return attr_getter(data) if msg_eval is None else attr_getter(msg_eval(data))

    return func


def value_transform(attrs):
    attrs = attrs

    class DictZipper(object):
        def __init__(self, attrs, vals):
            self.dct = dict(zip(attrs, vals))

        def __str__(self):
            return "\n" + yaml.dump(self.dct, allow_unicode=True, default_flow_style=False)

    def func(vals, _):
        return DictZipper(attrs, vals)

    return func


def extract_attrs(topic_with_attrs):
    if not topic_with_attrs.endswith(']'):
        return topic_with_attrs, None

    last_opening_bracket = topic_with_attrs.rfind('[')
    if last_opening_bracket == -1:
        return topic_with_attrs, None

    extract = re.sub(r"^.*\[(.*?)\][^\[]*$", r"\g<1>", topic_with_attrs)
    if not extract:
        return topic_with_attrs, None

    # check if the value inside brackets can be handled by rostopic (array slicing)
    try:
        _ = rostopic._get_array_index_or_slice_object(extract)
        return topic_with_attrs, None
    except AssertionError:
        pass

    # or a single field
    if ',' not in extract:
        # we just transform into a field that rostopic can resolve
        topic = topic_with_attrs[:last_opening_bracket]
        if not topic.endswith('/'):
            topic += '/'
        topic += extract

        return topic, None

    # check if they are valid attribute names?

    return topic_with_attrs[:last_opening_bracket], extract.split(',')


def spawn_subscriber(topic):
    # resolves namespace .. in this case making it global
    topic = rosgraph.names.script_resolve_name('', topic)

    # split attributes if any
    topic, attrs = extract_attrs(topic)

    # trailing '/' doesn't work well with rostopic
    topic = topic.rstrip('/')

    # TODO: remove blocking
    msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=True)
    if msg_class is None:
        return

    callback_echo = CallbackDiffEcho(topic)
    if attrs is not None:
        callback_echo.msg_eval = attr_eval(msg_eval, attrs)
        callback_echo.value_transform = value_transform(attrs)
    else:
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
                        raise rostopic.ROSTopicException(
                                "Cannot load message class for [%s]. Are your messages built?" % type_information)

    rospy.Subscriber(real_topic, msg_class, callback_echo.callback,
                     {'topic': topic, 'type_information': type_information})


def handle_args(args):
    if args.list_published_topics:
        rostopic._rostopic_list(None, verbose=True, publishers_only=True)
        return

    if not args.topics:
        return

    rostopic._check_master()

    rospy.init_node('ros_topics_watcher', anonymous=True)

    for topic in args.topics:
        threading.Thread(target=spawn_subscriber, args=(topic,)).start()

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
