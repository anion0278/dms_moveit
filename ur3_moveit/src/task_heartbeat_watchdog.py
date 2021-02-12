#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Empty
import setproctitle

import util_ros_process
import hmi_controller_disconnector
import config
setproctitle.setproctitle(config.hmi_watchdog_process)


class HeartbeatSender():
    def __init__(self, node_name):
        self.beat_pub = rospy.Publisher(node_name+config.heartbeat_topic, Empty, queue_size=1)

    def send_beat(self):
        self.beat_pub.publish()

class HeartbeatMonitor():
    def __init__(self, monitored_topics_arr, callback = None):
        rospy.init_node('watchdog', anonymous=True)
        self.timeout = rospy.Duration(0.6)
        self.callback = callback
        self.check_rate = rospy.Rate(5)
        self.previous_beat_monitors = dict()
        self.topic_monitors = dict()
        for topic_name in monitored_topics_arr:
            self.topic_monitors[topic_name] = rospy.Subscriber(topic_name, Empty, queue_size=1, callback=self.__on_beat, callback_args=topic_name)

    def run(self):
        print("Watchdog started")
        while not rospy.is_shutdown():
            self.check_rate.sleep()
            current_time = rospy.Time.now()
            for topic_name, topic_prev_msg_time in self.previous_beat_monitors.items():
                time_since_prev_beat = current_time - topic_prev_msg_time
                if time_since_prev_beat > self.timeout:
                    self.__on_timeout(topic_name)
                    
        print("Watchdog: ROS shutdown")
        self.__on_exit()

    def __on_beat(self, data, topic_name):
        current_time = rospy.Time.now()
        self.previous_beat_monitors[topic_name] = current_time
    
    def __on_exit(self):
        # hmi_controller_disconnector.restart_adapter()
        hmi_controller_disconnector.run_full_disconnection(use_exit=True)
        # sys.exit()

    def __on_timeout(self, timeout_topic_name):
        print("WATCHDOG - Topic has died: %s" % timeout_topic_name)
        if self.callback is not None:
            self.callback(timeout_topic_name)

def kill_ros_graph(topic_name):
    util_ros_process.kill_ros_graph()

if __name__ == "__main__":
    # util.print_all_args()
    hm = HeartbeatMonitor([config.hmi_right+config.heartbeat_topic, config.hmi_left+config.heartbeat_topic], callback = kill_ros_graph)
    hm.run()
