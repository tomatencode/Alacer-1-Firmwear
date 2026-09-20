#pragma once

#include <ArduinoEigen.h>

class IMURocketCoordinateConverter {
public:
    IMURocketCoordinateConverter(const Eigen::Vector3f& imuToRocketTranslation, const Eigen::Quaternionf& imuToRocketRotation);

    Eigen::Vector3f transformIMUPointToRocket(const Eigen::Vector3f& imuPoint_m) const;
    Eigen::Vector3f rotateIMUVectorToRocket(const Eigen::Vector3f& imuVector) const;

    Eigen::Vector3f rotateRocketVectorToIMU(const Eigen::Vector3f& rocketVector) const;
    Eigen::Vector3f transformRocketPointToIMU(const Eigen::Vector3f& rocketPoint_m) const;
private:
    Eigen::Vector3f _imuToRocketTranslation;
    Eigen::Quaternionf _imuToRocketRotation;

    Eigen::Quaternionf _rocketToIMURotation;
    Eigen::Vector3f _rocketToIMUTranslation;
};