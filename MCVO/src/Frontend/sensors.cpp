#include "sensors.h"

namespace MCVO
{
    // MCVOsensor base class implementation
    MCVOsensor::MCVOsensor(sensor_type stype,
                           string topic,
                           string name,
                           ros::NodeHandle *node,
                           Eigen::Matrix3d R,
                           Eigen::Vector3d T)
        : type(stype), topic(topic), name(name), frontend_node(node), ext_R(R), ext_T(T)
    {
        
    }

    // MCVOstereo implementation
    MCVOstereo::MCVOstereo(sensor_type type,
                           string left_img,
                           string right_img,
                           string name,
                           ros::NodeHandle *node,
                           Eigen::Matrix3d left_R,
                           Eigen::Vector3d left_T,
                           Eigen::Matrix3d right_R,
                           Eigen::Vector3d right_T,
                           double lfx, double lfy, double lcx, double lcy,
                           double rfx, double rfy, double rcx, double rcy,
                           bool lFISHEYE, bool rFISHEYE,
                           int lw, int lh, int rw, int rh)
        : MCVOsensor(type, left_img, name, node, left_R, left_T),
          lFISHEYE(lFISHEYE), rFISHEYE(rFISHEYE),
          lfx(lfx), lfy(lfy), lcx(lcx), lcy(lcy),
          rfx(rfx), rfy(rfy), rcx(rcx), rcy(rcy),
          lcol(lw), lrow(lh), rcol(rw), rrow(rh),
          lR(left_R), lT(left_T), rR(right_R), rT(right_T)
    {
        
    }

    bool MCVOstereo::setFisheye(string l_fisheye_path, string r_fisheye_path)
    {
        // Implementation for fisheye camera setup
        return true;
    }
}
