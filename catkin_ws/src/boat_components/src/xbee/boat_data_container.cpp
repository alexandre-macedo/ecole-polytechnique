#include "boat_data_container.h"

// Constructor
	BoatDataContainer::BoatDataContainer()
	{
		setCpuTemp(0);
	}

// Member Functions
	void setImu(sensor_msgs::Imu imu)
	{
		imu_=imu;
	}

	void setGps(sensor_msgs::NavSatFix gps)
	{
		gps_=gps;
	}

	void setCpuTemp(double cpu_temp)
	{
		cpu_temp_=cpu_temp;
	}
