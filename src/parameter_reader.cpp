#include "rgbdframe.h"
#include "parameter_reader.h"

rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS rgbd_tutor::ParameterReader::getCamera() const
{
    static rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = this->getData<double>("camera.fx");
    camera.fy = this->getData<double>("camera.fy");
    camera.cx = this->getData<double>("camera.cx");
    camera.cy = this->getData<double>("camera.cy");
    camera.d0 = this->getData<double>("camera.d0");
    camera.d1 = this->getData<double>("camera.d1");
    camera.d2 = this->getData<double>("camera.d2");
    camera.d3 = this->getData<double>("camera.d3");
    camera.d4 = this->getData<double>("camera.d4");
    camera.scale = this->getData<double>("camera.scale");

    return camera;
}
