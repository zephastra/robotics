// Recorded calibrated receiver cloud: actual PCL filters + plane segmentation.
// No simulator labels/body poses/order quantities are accepted.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>
#include <iostream>
#include <iomanip>

int main() {
    using Point=pcl::PointXYZ;
    auto input=pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    double x,y,z;
    while(std::cin>>x>>y>>z) {
        if(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)) {
            Point point;point.x=x;point.y=y;point.z=z;input->push_back(point);
        }
    }
    if(input->size()<60) {
        std::cout<<"{\"status\":\"UNKNOWN\",\"reason\":\"MISSING_FLOOR_POINTS\"}\n";
        return 0;
    }
    auto filtered=pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::VoxelGrid<Point> voxel;voxel.setInputCloud(input);
    voxel.setLeafSize(.001f,.001f,.001f);voxel.filter(*filtered);
    pcl::SACSegmentation<Point> segment;
    segment.setOptimizeCoefficients(true);
    segment.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segment.setMethodType(pcl::SAC_RANSAC);
    segment.setAxis(Eigen::Vector3f::UnitZ());segment.setEpsAngle(.05);
    segment.setDistanceThreshold(.001);segment.setMaxIterations(100);
    segment.setInputCloud(filtered);
    pcl::PointIndices inliers;pcl::ModelCoefficients coefficients;
    segment.segment(inliers,coefficients);
    if(inliers.indices.size()<60 || coefficients.values.size()!=4 ||
       std::abs(coefficients.values[2])<.99 ||
       double(inliers.indices.size())/filtered->size()<.9) {
        std::cout<<"{\"status\":\"UNKNOWN\",\"reason\":\"FLOOR_PLANE_UNRESOLVED\"}\n";
        return 0;
    }
    double mean_z=0;
    for(int i:inliers.indices)mean_z+=(*filtered)[i].z;
    mean_z/=inliers.indices.size();
    std::cout<<std::setprecision(12)<<"{\"status\":\"RESOLVED\",\"floor_z_m\":"<<mean_z
             <<",\"input_points\":"<<input->size()<<",\"filtered_points\":"<<filtered->size()
             <<",\"plane_inliers\":"<<inliers.indices.size()<<"}\n";
    return 0;
}
