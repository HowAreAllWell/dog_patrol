#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define PCL_NO_PRECOMPILE  // 允许自定义点类型
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 自定义包含 XYZ + RGB + Intensity + Curvature + Normals 的终极点云结构
struct PointXYZRGBI {
    PCL_ADD_POINT4D;  // 包含 x, y, z 坐标及 padding
    PCL_ADD_RGB;      // 包含 r, g, b, a 颜色信息
    float intensity;  // 包含激光雷达物理反射率
    float curvature;  // 包含高频时序补偿偏移
    float normal_x;
    float normal_y;
    float normal_z;

    inline Eigen::Map<const Eigen::Vector3f> getNormalVector3fMap() const {
        return (Eigen::Map<const Eigen::Vector3f>(&normal_x));
    }
    inline Eigen::Map<Eigen::Vector3f> getNormalVector3fMap() { return (Eigen::Map<Eigen::Vector3f>(&normal_x)); }
    inline Eigen::Map<const Eigen::Vector4f> getNormalVector4fMap() const {
        return (Eigen::Map<const Eigen::Vector4f>(&normal_x));
    }
    inline Eigen::Map<Eigen::Vector4f> getNormalVector4fMap() { return (Eigen::Map<Eigen::Vector4f>(&normal_x)); }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保内存对齐
} EIGEN_ALIGN16;

// 注册该点云类型，使其能被 PCL 和 ROS 2 序列化识别
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float, x, x)(float, y, y)(float, z, z)(uint32_t, rgba,
                                                                          rgba)(float, intensity,
                                                                                intensity)(float, curvature, curvature)(
                                      float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z))

typedef PointXYZRGBI PointType;
typedef PointXYZRGBI PointTypeRGB;
typedef PointXYZRGBI PointTypeRGBA;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudXYZRGBA;

typedef Eigen::Vector2f V2F;
typedef Eigen::Vector2d V2D;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

struct Pose6D {
    /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
    double offset_time;  // the offset time of IMU measurement w.r.t the first lidar point
    double acc[3];       // the preintegrated total acceleration (global frame) at the Lidar origin
    double gyr[3];       // the unbiased angular velocity (body frame) at the Lidar origin
    double vel[3];       // the preintegrated velocity (global frame) at the Lidar origin
    double pos[3];       // the preintegrated position (global frame) at the Lidar origin
    double rot[9];       // the preintegrated rotation (global frame) at the Lidar origin
};

#endif