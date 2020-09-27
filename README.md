# ROS Topics Watcher

Inspired by [py-trees-blackboard-watcher](https://py-trees-ros.readthedocs.io/en/devel/features.html#py-trees-blackboard-watcher), `ros-topics-watcher` is a wrap on top of [rostopic echo](http://wiki.ros.org/rostopic#rostopic_echo) tool, that prints out values only if they change. In addition it also allows subscribing to multiple topics

## Examples

```
>>> ros-topics-watcher -l

Published topics:
 * /foo [std_msgs/Header] 1 publisher
 * /bar [std_msgs/Header] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher

>> ros-topics-watcher /foo/seq
/foo/seq: 0
---

>> ros-topics-watcher /foo/seq /bar/stamp/nsecs
/foo/seq: 0
---

>> ros-topics-watcher /foo/seq /bar/stamp/nsecs
/foo/seq: 0
---
/bar/stamp/nsecs: 34739017
---
/bar/stamp/nsecs: 134696006
---
/bar/stamp/nsecs: 234863996
---
/bar/stamp/nsecs: 334772109
---
/bar/stamp/nsecs: 434758901
---
/bar/stamp/nsecs: 534750938
---
/bar/stamp/nsecs: 634763956
---

```