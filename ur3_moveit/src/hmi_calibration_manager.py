
#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import TriggerRequest, Trigger

import config


class SensorCalibration():
    def __init__(self, sys, gyro, acc, mag):
        self.sys = int(sys)
        self.gyro = int(gyro)
        self.acc = int(acc)
        self.mag = int(mag)
    
    @property
    def is_ready_for_use(self):
        #Therefore we recommend that as long as Magnetometer is 3/3, and Gyroscope is 3/3, the data can be trusted
        if self.gyro == 3 and self.mag == 3:
            return True
        return False
    
    def is_fully_calibrated(self):
        if self.sys == 3 and self.gyro == 3 and self.acc == 3 and self.mag == 3:
            return True
        return False

    def short_format(self):
        return "S{0}; G{1}; A{2}; M{3}".format(self.sys, self.gyro, self.acc, self.mag)

    def __str__(self):
        return "Sys: {0}; Gyro: {1}; Acc: {2}; Mag: {3}".format(self.sys, self.gyro, self.acc, self.mag)



if __name__ == "__main__":
    print("Requesting calibration...")
    
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
    print("Finished")



