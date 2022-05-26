#include <xsensdeviceapi.h>
//#include <xscontroller/xsdevice_def.h>
//#include <xstypes/xsdatapacket.h>

#include "hal_xsens_mti_620/xsens_mti620.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace hal
{

void cb(struct XsString const* s){
	std::cerr << s->c_str() << std::endl;
}

XsensMti620::XsensMti620(const rclcpp::NodeOptions & options) : Node("XsensMti620", options)
{
	topic = declare_parameter<std::string>("topic", "/imu/data");
	frame = declare_parameter<std::string>("frame", "imu");
	port = declare_parameter<std::string>("port", "/dev/ttyS2");
	baudrate = declare_parameter<int>("baudrate", 115200);
	freq = declare_parameter<int>("frequency", 100);
	lin_acc = declare_parameter<bool>("lin_acc", false);

	control = XsControl::construct();
	assert(control != 0);

	XsVersion version;
	xdaVersion(&version);
	RCLCPP_INFO(get_logger(), "Using XDA version %s", version.toString().c_str());

	auto _port = XsString(port);
	auto _baud = XsBaud::numericToRate(baudrate);
	if(_baud == XBR_Invalid){
		RCLCPP_ERROR(get_logger(),"Invalid baudrate. Aborting!!");
		std::abort();
	}

	//XsScanner::setScanLogCallback(&cb);

	RCLCPP_INFO(get_logger(), "Scanning port %s at baudrate %d", port.c_str(), XsBaud::rateToNumeric(_baud));
	mtPort = XsScanner::scanPort(_port, _baud);
	if(mtPort.empty()){
		RCLCPP_ERROR(get_logger(),"No MTi devide found. Aborting!!");
		std::abort();
	}

	auto did = mtPort.deviceId();
	RCLCPP_INFO(get_logger(), "Found Device with: ");
	RCLCPP_INFO(get_logger(), "	Devide ID: %s", did.toString().c_str());
	RCLCPP_INFO(get_logger(), "	Port Name: %s", mtPort.portName().c_str());
	RCLCPP_INFO(get_logger(), "	Baudrate : %d", XsBaud::rateToNumeric(mtPort.baudrate()));

	RCLCPP_INFO(get_logger(), "Opening Port ... ");
	if(!control->openPort(mtPort)){
		RCLCPP_ERROR(get_logger(),"Could Not Open the Port. Aborting!!");
		std::abort();
	}

	//Get the device Object:
	device = control->device(did);
	assert(device != 0);
	RCLCPP_INFO(get_logger(), "Device with ID: %s opened", device->deviceId().toString().c_str());

	//Create the Callback handler to device:

	device->addCallbackHandler(&callback);

	//Put the device in Configuration Mode:
	RCLCPP_INFO(get_logger(), "Putting the device in configuration mode.. ");
	if(!device->gotoConfig())
	{
		RCLCPP_ERROR(get_logger(), "Could not put the device into config mode. Aborting!!");
		std::abort();
	}

	if(!device->setOnboardFilterProfile("Robust/VRUAHS"))
	{
		RCLCPP_ERROR(get_logger(), "Could not set internal filter profile. Aborting!!");
		std::abort();
	}

	RCLCPP_INFO(get_logger(), "Configuring the device ...");
	auto configArray = XsOutputConfigurationArray();
	configArray.push_back(XsOutputConfiguration(XsDataIdentifier::XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XsDataIdentifier::XDI_SampleTimeFine, 0));

	if(device->deviceId().isVru() || device->deviceId().isAhrs() || device->deviceId().isGnss()){
		if (lin_acc){
			configArray.push_back(XsOutputConfiguration(XsDataIdentifier::XDI_Acceleration, freq));
		}
		configArray.push_back(XsOutputConfiguration(XsDataIdentifier::XDI_RateOfTurn, freq));
		configArray.push_back(XsOutputConfiguration(XsDataIdentifier::XDI_Quaternion, freq));
	} else {
		RCLCPP_ERROR(get_logger(), "Unknown device while configuring. Aborting!!");
		std::abort();
	}

	if(!device->setOutputConfiguration(configArray)){
		RCLCPP_ERROR(get_logger(), "Could not configure the device. Aborting!!");
		std::abort();
	}

	RCLCPP_INFO(get_logger(), "Setting the device in Measurement mode ...");
	if(!device->gotoMeasurement())
	{
		RCLCPP_ERROR(get_logger(), "Could not put the device into measurement mode. Aborting!!");
		std::abort();
	}

	imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(topic, 1);
	auto interval = 5ms;
	timer = create_wall_timer(interval, std::bind(&XsensMti620::run, this));
}

XsensMti620::~XsensMti620(){
	if (device != nullptr)
	{
		device->stopRecording();
		device->closeLogFile();
		device->removeCallbackHandler(&callback);
	}
	control->closePort(mtPort);
	control->destruct();
}

void XsensMti620::run(){
	auto data_packet = callback.next(std::chrono::milliseconds(2));
	if (!data_packet.second.empty())
	{
		auto quaternion = data_packet.second.orientationQuaternion();
		auto gyr = data_packet.second.calibratedGyroscopeData();
		auto acc = data_packet.second.calibratedAcceleration();

		//Publish Msg:
		std::unique_ptr<sensor_msgs::msg::Imu> pkt_imu(new sensor_msgs::msg::Imu);
		pkt_imu->header = std_msgs::msg::Header();
		pkt_imu->header.stamp = now();
		pkt_imu->header.frame_id = frame;

		pkt_imu->orientation.x = quaternion.x();
		pkt_imu->orientation.y = quaternion.y();
		pkt_imu->orientation.z = quaternion.z();
		pkt_imu->orientation.w = quaternion.w();
		pkt_imu->angular_velocity.x = gyr[0];
		pkt_imu->angular_velocity.y = gyr[1];
		pkt_imu->angular_velocity.z = gyr[2];
		if (lin_acc){
			pkt_imu->linear_acceleration.x = acc[0];
			pkt_imu->linear_acceleration.y = acc[1];
			pkt_imu->linear_acceleration.z = acc[2];
		}
		imu_publisher_->publish(std::move(pkt_imu));
	}
}

} //namespace hal
