#!/usr/bin/env pyhton3
import rclpy, os, numpy, sys, gc, time
from threading import Thread, Lock
import xsensdeviceapi as xda

from rclpy.node import Node
from sensor_msgs.msg import Imu

class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size = 5):
        xda.XsCallback.__init__(self)
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def getNextPacket(self):
        self.m_lock.acquire()
        assert(len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        self.m_lock.release()
        return oldest_packet

    def onLiveDataAvailable(self, dev, packet):
        self.m_lock.acquire()
        assert(packet != 0)
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()

class HALNode(Node):
	def __init__(self):
		super().__init__("hal_xsens_mti_620")
		
		self.declare_parameter('port', '/dev/ttyS2')
		self.declare_parameter('baudrate', '/dev/ttyS2')
		self.declare_parameter('imu_topic', '/imu/data')
		self.imu_publisher = self.create_publisher(Imu, self.get_parameter('imu_topic').value, 1)

	def run(self):

		control = xda.XsControl_construct()
		assert(control != 0)

		xdaVersion = xda.XsVersion()
		xda.xdaVersion(xdaVersion)
		self.get_logger().info("Using XDA version %s" % xdaVersion.toXsString())
       
		mtPort = xda.XsPortInfo()
		mtPort = xda.XsScanner_scanPort(self.get_parameter("port").value, xda.XBR_115k2)
		
		if mtPort.empty():
			raise RuntimeError("No MTi device found. Aborting.")

		did = mtPort.deviceId()
		print("Found a device with:")
		print(" Device ID: %s" % did.toXsString())
		print(" Port name: %s" % mtPort.portName())

		print("Opening port...")
		if not control.openPort(mtPort.portName(), mtPort.baudrate()):
			raise RuntimeError("Could not open port. Aborting.")

		# Get the device object
		device = control.device(did)
		assert(device != 0)

		self.get_logger().info("Device: %s, with ID: %s opened." % (device.productCode(), device.deviceId().toXsString()))

		# Create and attach callback handler to device
		callback = XdaCallback()
		device.addCallbackHandler(callback)

		# Put the device into configuration mode before configuring the device
		self.get_logger().info("Putting device into configuration mode...")
		if not device.gotoConfig():
			raise RuntimeError("Could not put device into configuration mode. Aborting.")

		'''
		profiles = device.availableOnboardFilterProfiles()
		for i in range(profiles.size()):
			profile = profiles[i]
			print(profile.kind(), profile.label())
		
		filter_profile = device.onboardFilterProfile()
		print(filter_profile.label())
		print(device.setOnboardFilterProfile("Robust/VRUAHS"))
		filter_profile = device.onboardFilterProfile()
		print(filter_profile.label())
		'''

		if not device.setOnboardFilterProfile("Robust/VRUAHS"):
			raise RuntimeError("Could not set internal filter profile. Aborting.")

		self.get_logger().info("Configuring the device...")
		configArray = xda.XsOutputConfigurationArray()
		configArray.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
		configArray.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))

		if device.deviceId().isVru() or device.deviceId().isAhrs() or device.deviceId().isGnss():
			configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
			configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
			configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
		else:
			raise RuntimeError("Unknown device while configuring. Aborting.")

		if not device.setOutputConfiguration(configArray):
			raise RuntimeError("Could not configure the device. Aborting.")

		self.get_logger().info("Putting device into measurement mode...")
		if not device.gotoMeasurement():
			raise RuntimeError("Could not put device into measurement mode. Aborting.")

		while True:
			if callback.packetAvailable():
				# Retrieve a packet
				packet = callback.getNextPacket()
				quaternion = packet.orientationQuaternion()
				gyr = packet.calibratedGyroscopeData()
				acc = packet.calibratedAcceleration()
				
				pkt_imu = Imu()
				pkt_imu.orientation.x = quaternion[0]
				pkt_imu.orientation.y = quaternion[1]
				pkt_imu.orientation.z = quaternion[2]
				pkt_imu.orientation.w = quaternion[3]
				pkt_imu.angular_velocity.x = gyr[0]
				pkt_imu.angular_velocity.y = gyr[1]
				pkt_imu.angular_velocity.z = gyr[2]
				pkt_imu.linear_acceleration.x = acc[0]
				pkt_imu.linear_acceleration.y = acc[1]
				pkt_imu.linear_acceleration.z = acc[2]

				self.imu_publisher.publish(pkt_imu)

##### Main function to loop
def main(args=None):
	for i in range(1):
		rclpy.init(args=args)
		node = HALNode()
		try:		
			node.run()
		except KeyboardInterrupt:
			print('Node hal_xsens_mti_620 stopped cleanly')
		except BaseException:
			print('Exception in Node hal_xsens_mti_620:', file=sys.stderr)
			raise
		finally:
			rclpy.shutdown()
		del(node)


##### Main Loop
if __name__ == "__main__":
	main()
