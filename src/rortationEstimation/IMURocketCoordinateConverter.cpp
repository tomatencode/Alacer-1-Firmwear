#include "IMURocketCoordinateConverter.hpp"

IMURocketCoordinateConverter::IMURocketCoordinateConverter(
    const Eigen::Vector3f& imuToRocketTranslation,
    const Eigen::Quaternionf& imuToRocketRotation)
    : _imuToRocketTranslation(imuToRocketTranslation),
      _imuToRocketRotation(imuToRocketRotation.normalized()),
      _rocketToIMURotation(_imuToRocketRotation.conjugate()),
      _rocketToIMUTranslation(-(_rocketToIMURotation * _imuToRocketTranslation))
{}

Eigen::Vector3f IMURocketCoordinateConverter::transformIMUPointToRocket(const Eigen::Vector3f& imuPoint_m) const {
    return _imuToRocketRotation * imuPoint_m + _imuToRocketTranslation;
}

Eigen::Vector3f IMURocketCoordinateConverter::rotateIMUVectorToRocket(const Eigen::Vector3f& imuVector) const {
    return _imuToRocketRotation * imuVector;
}

Eigen::Vector3f IMURocketCoordinateConverter::rotateRocketVectorToIMU(const Eigen::Vector3f& rocketVector) const {
    return _rocketToIMURotation * rocketVector;
}

Eigen::Vector3f IMURocketCoordinateConverter::transformRocketPointToIMU(const Eigen::Vector3f& rocketPoint_m) const {
    return _rocketToIMURotation * rocketPoint_m + _rocketToIMUTranslation;
}