#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

import Adafruit_FXOS8700
import Adafruit_FXAS21002C

def imu_talker():
    pub = rospy.Publisher('imu0', Imu, queue_size=10)
    rospy.init_node('imu0', anonymous=True)
    rate = rospy.Rate(200) # 10hz

    acc_magn_sensor = Adafruit_FXOS8700.FXOS8700()
    rospy.loginfo("acc magn ready")

    gyro_sensor = Adafruit_FXAS21002C.FXAS21002C()
    rospy.loginfo("gyro ready")

    while not rospy.is_shutdown():
        rosimu = Imu()

        gyro_x, gyro_y, gyro_z = gyro_sensor.gyroscope
        accel_x, accel_y, accel_z = acc_magn_sensor.accelerometer
        #mag_x, mag_y, mag_z = acc_magn_sensor.magnetometer

        rosimu.header.stamp = rospy.Time.now()
        rosimu.header.frame_id = "imu"

        rosimu.angular_velocity.x = gyro_x
        rosimu.angular_velocity.y = gyro_y 
        rosimu.angular_velocity.z = gyro_x

        rosimu.linear_acceleration.x = accel_x 
        rosimu.linear_acceleration.y = accel_y
        rosimu.linear_acceleration.z = accel_z

        #data_str = '\nAcceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z)
        #data_str += '\nGyroscope (rad/s): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(gyro_x, gyro_y, gyro_z)
        #rospy.loginfo(data_str)

        pub.publish(rosimu)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_talker()
    except rospy.ROSInterruptException:
        pass
