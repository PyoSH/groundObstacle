#include "tool.h"

namespace tool
{

Eigen::VectorXf eigen2xyzrpy(Eigen::Matrix4f mat)
{
  Eigen::VectorXf result(6);
  mat2xyzrpy(eigen2mat(mat), &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]);
  return result;
}

Eigen::Matrix4f xyzrpy2eigen(float x, float y, float z, float roll, float pitch, float yaw)
{
  Eigen::Matrix4f result =  mat2eigen(xyzrpy2mat(x,y,z,roll,pitch,yaw));
  return result;
}

Eigen::Matrix3f get_rotation_matrix(float roll, float pitch, float yaw) {
    // Rx: roll에 대한 회전 행렬
    Eigen::Matrix3f Rx;
    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

    // Ry: pitch에 대한 회전 행렬
    Eigen::Matrix3f Ry;
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    // Rz: yaw에 대한 회전 행렬
    Eigen::Matrix3f Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;

    // 전체 회전 행렬: Rz * Ry * Rx
    Eigen::Matrix3f rotation_matrix = Rz * Ry * Rx;

    return rotation_matrix;
}

void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
  // Create a segmentation object for the plane model
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);  // 조정 가능한 거리 임계값 (바닥 면 허용 오차)

  // ROI setting
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ROI(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.5, -0.2);
  // pass.setFilterLimitsNegative(true);
  // pass.filter(*pc_ROI);
  pass.filter(*output_cloud);

  // Perform segmentation to find inliers that represent the ground plane
  // seg.setInputCloud(pc_ROI);
  // seg.segment(*inliers, *coefficients);

  // if (inliers->indices.empty()) {
  //     std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  //     return;
  // }

  // Extract the points that are not part of the ground plane
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud(pc_ROI);
  // extract.setIndices(inliers);
  // extract.setNegative(true);  // true로 설정하여 평면 이외의 점들만 추출
  // extract.filter(*output_cloud);
}

void removeGroundPlaneWithNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, 
                                 float distance_threshold, float normal_threshold) {
  // Create a segmentation object for the plane model with normal constraints
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // ROI setting
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ROI(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.5, -0.2);
  pass.setFilterLimitsNegative(true);
  pass.filter(*pc_ROI);

  // Create a KD-Tree for the normal estimation
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setInputCloud(pc_ROI);
  ne.setKSearch(50);  // Set the number of nearest neighbors to use for normal estimation
  ne.compute(*cloud_normals);

  // Configure the segmentation object
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setInputCloud(pc_ROI);
  seg.setInputNormals(cloud_normals);
  
  // Set axis for the plane to be near the xz plane (i.e., y-axis normal)
  Eigen::Vector3f axis(0.0, 1.0, 0.0);  // y-axis direction
  seg.setAxis(axis);
  seg.setEpsAngle(normal_threshold);  // [rad] Allowable angle deviation from the axis

  // Perform segmentation
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(pc_ROI);
  extract.setIndices(inliers);
  extract.setNegative(true);  
  extract.filter(*output_cloud);
}

}