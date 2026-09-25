/*
  * Author: Christian Llanes, Georgia Institute of Technology, USA
  */

#include "crazysim_plugin.h"
#include <future>
#include <algorithm>
#include <unistd.h>

#include <gz/plugin/Register.hh>
GZ_ADD_PLUGIN(crazyflie_interface::GzCrazyflieInterface,
			  gz::sim::System,
			  crazyflie_interface::GzCrazyflieInterface::ISystemConfigure,
			  crazyflie_interface::GzCrazyflieInterface::ISystemPreUpdate,
			  crazyflie_interface::GzCrazyflieInterface::ISystemPostUpdate)
using namespace crazyflie_interface;

GzCrazyflieInterface::GzCrazyflieInterface() :
	namespace_(kDefaultNamespace),
	motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
	imu_sub_topic_(kDefaultImuTopic),
	magnetic_field_sub_topic_(kDefaultMagneticFieldTopic),
	barometer_sub_topic_(kDefaultFluidPressureTopic),
	odom_sub_topic_(kDefaultOdomTopic),
	cf_prefix(kDefaultCfPrefix),
	isPluginOn(true){
}

GzCrazyflieInterface::~GzCrazyflieInterface() {
	isPluginOn = false;
	cfLibAddrInitialized_ = false;
	socketInit = false;
	isInit = false;
	
	senderCfFirmwareThread.join();
	senderCfLibThread.join();
	receiverCfFirmwareThread.join();
	receiverCfLibThread.join();

	camera_cv.notify_all();
	if (senderCameraThread.joinable())
		senderCameraThread.join();
	if (fd_cam >= 0)
		close(fd_cam);
}

void GzCrazyflieInterface::Configure(const gz::sim::Entity &_entity,
						const std::shared_ptr<const sdf::Element> &_sdf,
						gz::sim::EntityComponentManager &_ecm,
						gz::sim::EventManager &_eventMgr) {
	gzdbg << __FUNCTION__ << "() called." << std::endl;
		
  	auto sdfClone = _sdf->Clone();

	/* Get Crazylfie SDF parameters */
	cffirm_addr = sdfClone->Get<std::string>("cffirm_addr");
	cffirm_port = sdfClone->Get<std::string>("cffirm_port");
	cflib_addr = sdfClone->Get<std::string>("cflib_addr");
	cflib_port = sdfClone->Get<std::string>("cflib_port");
	cf_prefix = sdfClone->Get<std::string>("cfPrefix");
	imu_sub_topic_ = sdfClone->Get<std::string>("imuSubTopic");
	magnetic_field_sub_topic_ = sdfClone->Get<std::string>("magSubTopic");
	barometer_sub_topic_ = sdfClone->Get<std::string>("baroSubTopic");
	cf_id_ = sdfClone->Get<int>("cfId");

	// ---------- Setup socket for Crazyflie firmware -----------
	std::string dest_addr_cfFirm = cffirm_addr;
	std::string dest_port_cfFirm = cffirm_port;
	// open socket in UDP mode
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		gzerr << "create CfFirm socket failed " << std::endl;
		return;
	}
	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	if (dest_addr_cfFirm == "INADDR_ANY")
		myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	else if (inet_addr(dest_addr_cfFirm.c_str()) == INADDR_NONE){
		gzerr << "Adress IP CfFirm invalid !" << std::endl;
		return;
	}else
		myaddr.sin_addr.s_addr = inet_addr(dest_addr_cfFirm.c_str());
	port = std::stoi(dest_port_cfFirm);
	myaddr.sin_port = htons(port);
	// Bind the socket
	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0){
		gzerr << "Bind socket CfFirm failed " << std::endl;
		return;
	}
	addrlen_rcv = sizeof(remaddr_rcv);
	

	// ---------- Setup socket for Crazyflie Python api -----------
	std::string dest_addr_cfLib = cflib_addr;
	std::string dest_port_cfLib = cflib_port;
	// open CfLib socket in UDP mode
	if ((fd_cfLib = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		gzerr << "create CfLib socket failed " << std::endl;
		return;
	}
	memset((char *)&myaddr_CfLib, 0, sizeof(myaddr_CfLib));
	myaddr_CfLib.sin_family = AF_INET;
	if (dest_addr_cfLib == "INADDR_ANY")
		myaddr_CfLib.sin_addr.s_addr = htonl(INADDR_ANY);
	else if (inet_addr(dest_addr_cfLib.c_str()) == INADDR_NONE){
		gzerr << "Adress IP CfLib invalid !" << std::endl;
		return;
	}else
		myaddr_CfLib.sin_addr.s_addr = inet_addr(dest_addr_cfLib.c_str());
	port_cfLib = std::stoi(dest_port_cfLib);
	myaddr_CfLib.sin_port = htons(port_cfLib);
	// Bind the socket
	if (bind(fd_cfLib, (struct sockaddr *)&myaddr_CfLib, sizeof(myaddr_CfLib)) < 0){
		gzerr << "Bind socket CfLib failed " << std::endl;
		return;
	}
	addrlen_rcv_cfLib = sizeof(remaddr_rcv_cfLib);

	imu_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	barometer_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	odom_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	cflib_to_firmware_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	firmware_to_cflib_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);

	addrlen = sizeof(remaddr);
	addrlen_cfLib = sizeof(remaddr_cfLib);
	isInit = false;
	socketInit = false;
	cfLibAddrInitialized_ = false;
	m_motor_command_.m1 = 0;
	m_motor_command_.m2 = 0;
	m_motor_command_.m3 = 0;
	m_motor_command_.m4 = 0;

	// ---------- Setup socket for AI-deck camera frames (only if the model has a camera) -----------
	if (sdfClone->HasElement("camFramePort")) {
		if ((fd_cam = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			gzerr << "create camera socket failed " << std::endl;
		} else {
			memset((char *)&camaddr, 0, sizeof(camaddr));
			camaddr.sin_family = AF_INET;
			camaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
			camaddr.sin_port = htons(sdfClone->Get<int>("camFramePort"));
			camera_enabled_ = true;
		}
	}

	receiverCfFirmwareThread = std::thread(&GzCrazyflieInterface::recvCfFirmwareThread, this);
	receiverCfLibThread = std::thread(&GzCrazyflieInterface::recvCfLibThread, this);
	senderCfFirmwareThread = std::thread(&GzCrazyflieInterface::sendCfFirmwareThread, this);
	senderCfLibThread = std::thread(&GzCrazyflieInterface::sendCfLibThread, this);
	if (camera_enabled_)
		senderCameraThread = std::thread(&GzCrazyflieInterface::sendCameraThread, this);
	initializeSubsAndPub();
}

void GzCrazyflieInterface::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
	writeMotors();
}

void GzCrazyflieInterface::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) {

}

void GzCrazyflieInterface::ImuCallback(const gz::msgs::IMU& imu_msg) {
	struct imu_s m_imu_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_GYRO_ACC_SIM,
		.acc = { static_cast<int16_t>(imu_msg.linear_acceleration().x()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg.linear_acceleration().y()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg.linear_acceleration().z()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF)},
		.gyro = {static_cast<int16_t>(imu_msg.angular_velocity().x()	   	/ 	SENSORS_DEG_PER_LSB_CFG	 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg.angular_velocity().y()    	/ 	SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg.angular_velocity().z()    	/	SENSORS_DEG_PER_LSB_CFG		/ DEG_TO_RAD_CF)}};

	crtpPacket_t msg;
	msg.size = sizeof(m_imu_info);
	memcpy(msg.raw, (const uint8_t*) &m_imu_info , sizeof(m_imu_info));
	imu_queue.enqueue(msg);
}

void GzCrazyflieInterface::BarometerCallback(const gz::msgs::FluidPressure& air_pressure_msg) {
	double temperature_at_altitude_kelvin = kSeaLevelTempKelvin * exp(- log(air_pressure_msg.pressure() / kPressureOneAtmospherePascals)/kAirConstantDimensionless);
	double height_geopotential_m = (kSeaLevelTempKelvin - temperature_at_altitude_kelvin)/kTempLapseKelvinPerMeter;
	double height_geometric_m = height_geopotential_m * kEarthRadiusMeters / (kEarthRadiusMeters - height_geopotential_m);
	
	struct baro_s m_baro_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_BARO_SIM,
		.pressure = static_cast<float>(air_pressure_msg.pressure() * 0.01), // Convert in mbar
		.temperature = static_cast<float>(temperature_at_altitude_kelvin - 273.15),
		.asl = static_cast<float>(height_geometric_m)
	};

	crtpPacket_t msg;
	msg.size = sizeof(m_baro_info);
	memcpy(msg.raw, (const uint8_t*) &m_baro_info , sizeof(m_baro_info));
	barometer_queue.enqueue(msg);
}

void GzCrazyflieInterface::OdomCallback(const gz::msgs::Odometry& odom_msg) {
	// Just need to transfer to the fcu the external position
	struct CrtpExtPose_s ext_pose  = {
		.header = crtp(CRTP_PORT_LOCALIZATION, CRTP_LOC_CHANNEL_GEN_LOC),
		.id = CRTP_GEN_LOC_ID_EXT_POS,
		.x = static_cast<float>(odom_msg.pose().position().x()),
		.y = static_cast<float>(odom_msg.pose().position().y()), 
		.z = static_cast<float>(odom_msg.pose().position().z()),
		.qx = static_cast<float>(odom_msg.pose().orientation().x()),
		.qy = static_cast<float>(odom_msg.pose().orientation().y()),
		.qz = static_cast<float>(odom_msg.pose().orientation().z()),
		.qw = static_cast<float>(odom_msg.pose().orientation().w())
	}; 
	
	crtpPacket_t msg;
	msg.size = sizeof(ext_pose);
	memcpy(msg.raw, (const uint8_t*) &ext_pose , sizeof(ext_pose));
	odom_queue.enqueue(msg);
}

void GzCrazyflieInterface::CameraCallback(const gz::msgs::Image& image_msg) {
	if (image_msg.pixel_format_type() != gz::msgs::L_INT8) {
		if (!camera_format_warned) {
			gzwarn << "AI-deck camera expects 8-bit grayscale (L8) images, dropping frames" << std::endl;
			camera_format_warned = true;
		}
		return;
	}
	const uint32_t width = image_msg.width();
	const uint32_t height = image_msg.height();
	const uint32_t step = image_msg.step() ? image_msg.step() : width;
	const std::string& data = image_msg.data();
	if (data.size() < static_cast<size_t>(step) * height)
		return;

	{
		std::lock_guard<std::mutex> lock(camera_mutex);
		camera_frame.resize(static_cast<size_t>(width) * height);
		for (uint32_t row = 0; row < height; row++)
			memcpy(&camera_frame[static_cast<size_t>(row) * width], data.data() + static_cast<size_t>(row) * step, width);
		camera_width = width;
		camera_height = height;
		camera_frame_ready = true;
	}
	camera_cv.notify_one();
}

bool GzCrazyflieInterface::sendCfFirmware(const uint8_t* data , uint32_t length) {
	int transferred = sendto(fd, data, length, 0, (struct sockaddr *) &(remaddr), addrlen);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
    return true;
}

bool GzCrazyflieInterface::sendCfLib(const uint8_t* data , uint32_t length) {	
	int transferred = sendto(fd_cfLib, data, length, 0, (struct sockaddr *) &(remaddr_cfLib), addrlen_cfLib);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
    return true;
}

void GzCrazyflieInterface::recvCfFirmwareThread() {
	uint8_t buf[32];
	while(isPluginOn)
	{
		int len = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv, &addrlen_rcv);
		if (len <= 0 )
			continue;
		if (!socketInit && buf[0] == 0xF3 && len == 1){
			gzmsg << "Received firmware handshake message..." << std::endl;
			remaddr = remaddr_rcv;
			addrlen = addrlen_rcv;
			socketInit = true;
			// Send response to SITL instance
			uint8_t data[1] = {0xF3};
			sendCfFirmware(data , sizeof(data));
		}
		else if (socketInit && (crtp(buf[0]) == crtp(0x09,0)) && (len == 9)) // Motor command message
			handleMotorsMessage(&buf[0]);
		else if (socketInit && cfLibAddrInitialized_){
			crtpPacket_t packet;
			packet.size = len - 1;
			memcpy(&packet.raw[0] , &buf[0], sizeof(uint8_t) * len);
			bool succeeded  = firmware_to_cflib_queue.enqueue(packet);
			if(!succeeded)
				gzmsg << "Failed to enqueue firmware_to_cflib_queue in recvCfFirmwareThread()." << std::endl;
		}
	}
}

void GzCrazyflieInterface::recvCfLibThread() {
	uint8_t buf[32];
	while(isPluginOn)
	{
		if (!socketInit)
			continue;
		int len = recvfrom(fd_cfLib, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv_cfLib, &addrlen_rcv_cfLib);
		if (len <= 0 )
			continue;

		// Update cflib address on every received packet (activity-based detection)
		remaddr_cfLib = remaddr_rcv_cfLib;
		addrlen_cfLib = addrlen_rcv_cfLib;
		cfLibAddrInitialized_ = true;

		// Echo back null CRTP packets (0xFF) for scan/ping
		if (buf[0] == 0xFF && len == 1) {
			uint8_t echo[1] = {0xFF};
			sendCfLib(echo, sizeof(echo));
		}

		// Forward all packets to firmware
		recvCfLib(&buf[0], len);
	}
}

void GzCrazyflieInterface::sendCfFirmwareThread() {
	crtpPacket_t msgs[10];
	std::pair<std::chrono::steady_clock::duration, crtpPacket_t> stamp_msg_pair;
	while(isPluginOn){
		if(!socketInit)
			continue;

		size_t imu_msg_count = imu_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != imu_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t baro_msg_count = barometer_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != baro_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t odom_msg_count = odom_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != odom_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t cflib_msg_count = cflib_to_firmware_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != cflib_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}
	}
}

void GzCrazyflieInterface::sendCfLibThread() {
	crtpPacket_t msgs[10];
	while(isPluginOn){
		if(!socketInit || !cfLibAddrInitialized_)
			continue;

		size_t cflib_msg_count = firmware_to_cflib_queue.wait_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != cflib_msg_count; i++) {
			sendCfLib(msgs[i].raw, msgs[i].size+1);
		}
	}
}

void GzCrazyflieInterface::sendCameraThread() {
	std::vector<uint8_t> frame;
	std::vector<uint8_t> packet(8 + kCameraChunkSize);
	while(isPluginOn){
		uint16_t width, height;
		{
			std::unique_lock<std::mutex> lock(camera_mutex);
			camera_cv.wait_for(lock, std::chrono::milliseconds(100), [this] { return camera_frame_ready || !isPluginOn; });
			if (!camera_frame_ready)
				continue;
			frame.swap(camera_frame);
			width = camera_width;
			height = camera_height;
			camera_frame_ready = false;
		}

		// Chunk format read by crazysim_cpx.py: [seq:u16][total:u16][width:u16][height:u16][pixels], little endian
		const uint16_t total = (frame.size() + kCameraChunkSize - 1) / kCameraChunkSize;
		for (uint16_t seq = 0; seq < total; seq++) {
			const size_t offset = static_cast<size_t>(seq) * kCameraChunkSize;
			const size_t len = std::min(kCameraChunkSize, frame.size() - offset);
			const uint16_t header[4] = {seq, total, width, height};
			for (int i = 0; i < 4; i++) {
				packet[2 * i] = header[i] & 0xFF;
				packet[2 * i + 1] = header[i] >> 8;
			}
			memcpy(&packet[8], frame.data() + offset, len);
			sendto(fd_cam, packet.data(), 8 + len, 0, (struct sockaddr *) &camaddr, sizeof(camaddr));
		}
	}
}


void GzCrazyflieInterface::initializeSubsAndPub() {
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + imu_sub_topic_, &GzCrazyflieInterface::ImuCallback , this);
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + barometer_sub_topic_, &GzCrazyflieInterface::BarometerCallback, this );
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + odom_sub_topic_, &GzCrazyflieInterface::OdomCallback, this );
	if (camera_enabled_) {
		node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + kDefaultCameraTopic, &GzCrazyflieInterface::CameraCallback, this );
		gzmsg << "Forwarding camera frames to udp://127.0.0.1:" << ntohs(camaddr.sin_port) << std::endl;
	}

	motor_velocity_reference_pub_ = node_.Advertise<gz::msgs::Actuators>(cf_prefix + "_" + std::to_string(cf_id_) + motor_velocity_reference_pub_topic_);
	
	isInit = true;
	gzmsg << "Init subs and Pubs done : " << std::endl;
}

void GzCrazyflieInterface::handleMotorsMessage(const uint8_t* data) {
	crtpMotorsDataResponse* motorsData = (crtpMotorsDataResponse *) data;
	{
		std::unique_lock<std::mutex> mlock(motors_mutex);
		m_motor_command_.m1 = PWM2OMEGA(motorsData->m1);
		m_motor_command_.m2 = PWM2OMEGA(motorsData->m2);
		m_motor_command_.m3 = PWM2OMEGA(motorsData->m3);
		m_motor_command_.m4 = PWM2OMEGA(motorsData->m4);
	}

}

void GzCrazyflieInterface::recvCfLib(uint8_t* data , int len) {
	crtpPacket_t packet;
	packet.size = len - 1;
	memcpy(&packet.raw[0] , data , sizeof(uint8_t) * len);
	bool succeeded  = cflib_to_firmware_queue.enqueue(packet);
	if(!succeeded)
		gzmsg << "Failed recvCfLib" << std::endl;
}


void GzCrazyflieInterface::writeMotors() {
	m_motor_speed.clear_velocity();
	motors_mutex.lock();
	m_motor_speed.add_velocity(m_motor_command_.m1); // 0
	m_motor_speed.add_velocity(m_motor_command_.m2); // 1
	m_motor_speed.add_velocity(m_motor_command_.m3); // 2
	m_motor_speed.add_velocity(m_motor_command_.m4); // 3
	motors_mutex.unlock();
	motor_velocity_reference_pub_.Publish(m_motor_speed);
}