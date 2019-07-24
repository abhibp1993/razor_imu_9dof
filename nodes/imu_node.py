import math
import serial
import sys
import time
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from std_msgs.msg import String


ROS2_IMU_NODE_NAME = "imu_node"
ROS2_IMU_TOPIC_NAME = "imu"
ROS2_DIAGNOSTIC_NODE_NAME = "diagnostics_node"
ROS2_DIAGNOSTIC_TOPIC_NAME = "diagnostics"


class RazorIMU(object):
    """
    Handles serial communication with RazorIMU and exposes the ROS2 topics
    """
    def __init__(self, serial_port="/dev/ttyACM0"):

        # Create serial object
        self.serial_port = serial_port
        self.serial = self.configure_serial_port()

        # Create IMU nodes
        self.node_imu = rclpy.create_node(node_name=ROS2_IMU_NODE_NAME)
        self.node_diag = rclpy.create_node(node_name=ROS2_DIAGNOSTIC_NODE_NAME)

        # Create IMU publishers
        self.pub_imu = self.node_imu.create_publisher(msg_type=Imu, topic=ROS2_IMU_TOPIC_NAME)
        self.pub_diag = self.node_imu.create_publisher(msg_type=DiagnosticArray, topic=ROS2_DIAGNOSTIC_TOPIC_NAME)

        # Accelerometer calibration values
        self.accel_x_min = -250.0
        self.accel_x_max = 250.0
        self.accel_y_min = -250.0
        self.accel_y_max = 250.0
        self.accel_z_min = -250.0
        self.accel_z_max = 250.0

        # Gyrometer calibration values
        self.gyro_average_offset_x = 0.0
        self.gyro_average_offset_y = 0.0
        self.gyro_average_offset_z = 0.0

        # Magnetometer calibration values
        self.calibration_magn_use_extended = False
        self.magn_x_min = -600.0
        self.magn_x_max = 600.0
        self.magn_y_min = -600.0
        self.magn_y_max = 600.0
        self.magn_z_min = -600.0
        self.magn_z_max = 600.0
        self.magn_ellipsoid_center = [0, 0, 0]
        self.magn_ellipsoid_transform = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        # Yaw calibration
        self.imu_yaw_calibration = 0.0

        # Linear acceleration scaling factor
        self.linear_acceleration_scale = 9.806 / 256.0

        # Message count tracker
        self.msgs_sent = 0

    def write(self, cmd: str):
        self.serial.write((cmd + chr(13)).encode())

    def readline(self):
        return self.serial.readline().decode("UTF-8")

    def readlines(self):
        return [line.decode("UTF-8") for line in self.serial.readlines()]

    def flush(self):
        self.serial.readlines()

    def calibrate(self):
        # Wait for IMU to boot completely.
        time.sleep(5)

        # Stop data stream from IMU
        self.write('#o0')

        # Flush buffer
        self.flush()

        # Calibrate accelerometer
        print("Calibrating accelerometer on Razor IMU board...")
        self.write('#caxm' + str(self.accel_x_min))
        self.write('#caxM' + str(self.accel_x_max))
        self.write('#caym' + str(self.accel_y_min))
        self.write('#cayM' + str(self.accel_y_max))
        self.write('#cazm' + str(self.accel_z_min))
        self.write('#cazM' + str(self.accel_z_max))

        # Calibrate gyrometer
        print("Calibrating gyrometer on Razor IMU board...")
        self.write('#cgx' + str(self.gyro_average_offset_x))
        self.write('#cgy' + str(self.gyro_average_offset_y))
        self.write('#cgz' + str(self.gyro_average_offset_z))

        # Calibrate magnetometer
        print("Calibrating magnetometer on Razor IMU board...")
        if not self.calibration_magn_use_extended:
            self.write('#cmxm' + str(self.magn_x_min))
            self.write('#cmxM' + str(self.magn_x_max))
            self.write('#cmym' + str(self.magn_y_min))
            self.write('#cmyM' + str(self.magn_y_max))
            self.write('#cmzm' + str(self.magn_z_min))
            self.write('#cmzM' + str(self.magn_z_max))
        else:
            self.write('#ccx' + str(self.magn_ellipsoid_center[0]))
            self.write('#ccy' + str(self.magn_ellipsoid_center[1]))
            self.write('#ccz' + str(self.magn_ellipsoid_center[2]))
            self.write('#ctxX' + str(self.magn_ellipsoid_transform[0][0]))
            self.write('#ctxY' + str(self.magn_ellipsoid_transform[0][1]))
            self.write('#ctxZ' + str(self.magn_ellipsoid_transform[0][2]))
            self.write('#ctyX' + str(self.magn_ellipsoid_transform[1][0]))
            self.write('#ctyY' + str(self.magn_ellipsoid_transform[1][1]))
            self.write('#ctyZ' + str(self.magn_ellipsoid_transform[1][2]))
            self.write('#ctzX' + str(self.magn_ellipsoid_transform[2][0]))
            self.write('#ctzY' + str(self.magn_ellipsoid_transform[2][1]))
            self.write('#ctzZ' + str(self.magn_ellipsoid_transform[2][2]))

        # Print calibration values for feedback to user
        self.flush()
        self.write('#p')
        data = self.readlines()
        data_print = "Printing set calibration values:\r\n"
        for line in data:
            data_print += line
        print(data_print)

        # Stop data stream from IMU
        self.write('#o0')
        self.flush()

    def configure_serial_port(self):
        try:
            ser = serial.Serial(port=self.serial_port, baudrate=57600, timeout=1)
            print("Created serial object...")
            return ser

        except serial.serialutil.SerialException:
            print("IMU not found at port " + self.serial_port + ". Did you specify the correct port?")
            sys.exit(0)

    def destroy_serial_port(self):
        self.serial.close()
        print("Serial port closed... ")

    def loop(self):
        # Start the streaming
        self.write('#ox')
        self.write('#o1')

        # Discard 200 inputs
        print("Flushing first 200 IMU entries...")
        for _ in range(0, 200):
            self.readline()

        # Start the data collection and publish loop
        print("Publishing IMU data...")
        while rclpy.ok():

            # Get data from IMU board
            orientation, linear_acceleration, angular_velocity = self.get_imu_reading()
            if None in orientation:
                continue

            # Publish the data to ROS2 topics
            self.publish_topic_imu(orientation, linear_acceleration, angular_velocity)
            self.publish_topic_diag()

    def get_imu_reading(self):
        # Initialize values
        roll = pitch = yaw = None
        linear_acceleration = None
        angular_velocity = None

        # Get one reading from IMU
        line = self.readline()

        # Strip initial signature of packet
        line = line.replace("#YPRAG=", "")
        words = str.split(line, ",")
        print(line)

        # Extract all numeric values as string of values
        if len(words) == 9:
            values = [float(value) for value in words]
            # Extract roll
            roll = math.radians(values[2])

            # Extract pitch
            pitch = math.radians(-values[1])                            # in ROS y axis points left (see REP 103)

            # Extract yaw
            yaw_deg = -values[0] + self.imu_yaw_calibration             # in ROS z axis points up (see REP 103)

            if yaw_deg > 180.0:
                yaw_deg = yaw_deg - 360.0

            if yaw_deg < -180.0:
                yaw_deg = yaw_deg + 360.0

            yaw = math.radians(yaw_deg)

            linear_acceleration = [-self.linear_acceleration_scale * values[3],
                                   self.linear_acceleration_scale * values[4],
                                   self.linear_acceleration_scale * values[5]
                                   ]

            angular_velocity = [values[6], -values[7], -values[8]]

        return (roll, yaw, pitch), linear_acceleration, angular_velocity

    def publish_topic_imu(self, orientation, linear_acceleration, angular_velocity):
        # Convert RPY -> quaternion
        rot = Rotation.from_euler('xyz', orientation, degrees=False)
        quat = rot.as_quat()

        imu_msg = Imu()
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]

        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]

        self.pub_imu.publish(imu_msg)

    def publish_topic_diag(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    imu = RazorIMU(serial_port="/dev/ttyACM0")

    try:
        imu.calibrate()
        imu.loop()

    finally:
        imu.destroy_serial_port()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
