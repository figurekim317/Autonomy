#ifndef __DBSCAN_PHASE_H__
#define __DBSCAN_PHASE_H__

#include <cmath>
#include <pcl/kdtree/kdtree_flann.h>

#include "util/range_util.h"

constexpr int UNCLASSIFIED = -1;
constexpr int CORE_POINT = 1;
constexpr int BORDER_POINT = 2;
constexpr int NOISE = -2;
constexpr int SUCCESS = 0;
constexpr int FAILURE = -3;

class DBSCAN
{
public:
    DBSCAN(unsigned int min_pts, float eps, pcl::PointCloud<pcl::PointXYZI>::Ptr &points);

    virtual ~DBSCAN();

    int run_obstacles();

    int run_curb();

    std::vector<int> calculate_cluster(pcl::PointXYZI &point);

    int expand_cluster_obstacles(pcl::PointXYZI &point, int intensity);

    int expand_cluster_curb(pcl::PointXYZI &point, int intensity);

    int get_total_pointSize() { return m_point_size; };
    int get_minimum_clusterSize() { return m_min_points; };
    int get_epsilon_size() { return m_epsilon; };
    int get_num_of_object() { return final_intensity; };

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_points;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    unsigned int m_point_size;
    unsigned int final_intensity;
    unsigned int m_min_points;
    float m_epsilon;
};

#endif // __DBSCAN_PHASE_H__
