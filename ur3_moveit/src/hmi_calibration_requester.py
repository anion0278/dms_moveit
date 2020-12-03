#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import TriggerRequest, Trigger

import config

if __name__ == "__main__":
    service_ids = ["/hmi_glove_left" + config.calibr_service, "/hmi_glove_right" + config.calibr_service]
    services = []
    for ser in service_ids:
        try:
            rospy.wait_for_service(ser, timeout=0.1)
            services.append(rospy.ServiceProxy(ser, Trigger))
        except Exception as e: 
            print("Could not find %s service" % ser)

    for ser in services:
        result = ser(TriggerRequest())
        # if result.success:
        print(result.message)
