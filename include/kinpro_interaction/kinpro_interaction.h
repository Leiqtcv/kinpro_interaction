/* *****************************************************************
 *
 * Copyright (c) 2014,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/
/**
 * @file   mainwindow.cpp
 * @author Lennart Claassen
 * @date   Sep 08 2014
 *
 * @brief  This file contains a class for the kinect projector gui window
 */
#ifndef _KINPRO_INTERACTION_H
#define _KINPRO_INTERACTION_H

#include <ros/ros.h>
#include <kinpro_interaction/line.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/common/centroid.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/extract_indices.h>

class kinproInteractor{

public:
    kinproInteractor();

    ~kinproInteractor();

    pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB>::Ptr c;
    pcl::visualization::PCLVisualizer *vis;
    pcl::visualization::CloudViewer *viewer;

    void run();

private:

    ros::NodeHandle* nh;
    ros::Subscriber pc_sub;
    ros::Publisher linePub;

    double filterMin;
    double filterMax;
    double planeDistThresh;
    bool useConcaveHull;
    double concaveAlpha;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc;
    std::string pointCloudTopic;

    void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);
    void displayCloud(pcl::PointCloud<pcl::PointXYZRGB> &pc, std::string id = std::string("cloud"));
    void publishLine(Eigen::Vector4f start, Eigen::Vector4f end, float length = 2.0);
};


#endif
