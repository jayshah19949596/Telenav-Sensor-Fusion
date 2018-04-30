#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <stdint-gcc.h>
// #include "Dense"
#include "Eigen/Dense"

class MeasurementPackage {
public:
	long long  timestamp_;

	enum SensorType {
		LASER, RADAR
	} sensor_type_;

	Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);

};

#endif /* MEASUREMENT_PACKAGE_H_ */