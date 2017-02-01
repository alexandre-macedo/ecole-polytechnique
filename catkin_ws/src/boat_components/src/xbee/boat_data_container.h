//=================================
// include guard
#ifndef BOAT_DATA_CONTAINER_H
#define BOAT_DATA_CONTAINER_H

//=================================
// included dependencies
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

//=================================
class BoatDataContainer
{
private:
	sensor_msgs::Imu imu_;
	sensor_msgs::NavSatFix gps_;
	double cpu_temp_;

public:
	BoatDataContainer();

	void setImu(sensor_msgs::Imu imu);
	void setGps(sensor_msgs::NavSatFix gps);
	void setCpuTemp(double cpu_temp);

	sensor_msgs::Imu getImu() { return imu_};
	sensor_msgs::NavSatFix getGps() { return gps_};
	double getCpuTemp() { return cpu_temp_};
};

#endif 