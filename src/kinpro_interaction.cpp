#include <kinpro_interaction/kinpro_interaction.h>

using namespace std;

kinproInteractor::kinproInteractor() {

    this->nh        = new ros::NodeHandle;

    nh->param<std::string>("cloud_in",   pointCloudTopic,    "/camera/depth_registered/points");
    nh->param<double>("kinpro_interaction/filter_min", m_filterMin, 0.0);
    nh->param<double>("kinpro_interaction/filter_max", m_filterMax, 0.3);
    nh->param<bool>("kinpro_interaction/use_concave_hull", m_useConcaveHull, true);
    nh->param<double>("kinpro_interaction/concave_alpha", m_concaveAlpha, 0.01);
    nh->param<double>("kinpro_interaction/plane_dist_thresh", m_planeDistThresh, 0.3);
    nh->param<double>("kinpro_interaction/line_length_thresh", m_lineLengthTresh, 0.1);
    nh->param<double>("kinpro_interaction/moving_average_size", m_averageSize, 10);
    nh->param<double>("kinpro_interaction/max_point_distance", m_maxPointDistance, 0.05);


    pc_sub   = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGB> >(pointCloudTopic, 2, &kinproInteractor::pointcloudCallback, this);

    linePub = nh->advertise< kinpro_interaction::line >("/line", 1, this);

    m_lastCentroid = Eigen::Vector4f(0,0,0,1);
    m_lastTip = Eigen::Vector4f(0,0,0,1);

    vis = new pcl::visualization::PCLVisualizer("vis", true);
    vis->setBackgroundColor(0.2, 0.2, 0.2);
    vis->addCoordinateSystem(0.5, 0.0, 0.0, 0.0);
}

kinproInteractor::~kinproInteractor() {

}

void kinproInteractor::run() {
    ros::Rate rate(30);

    while (!vis->wasStopped() && ros::ok()){
       ros::spinOnce();
       vis->spinOnce();
       rate.sleep();
    }
    cout << "stopped!" << endl;
}

void kinproInteractor::pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {

    //filter the input cloud to contain only the values inside of reach of the user
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (msg->makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ((float)m_filterMin, (float)m_filterMax);
    pass.filter(pc);

    //display the filtered cloud
//    this->displayCloud(pc, "msg");

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    //segmentate the plane of the pointing device to speed up next calculations
    if(!pc.points.empty()) {
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (m_planeDistThresh);

        seg.setInputCloud (pc.makeShared());
        seg.segment (*inliers, *coefficients);
    }

    //evaluate if a plane was found
    if(!inliers->indices.empty()) {

        //project the model inliers
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setIndices (inliers);
        proj.setInputCloud (pc.makeShared());
        proj.setModelCoefficients (coefficients);
        proj.filter (pc);

        //create a concave or convex hull representation of the projected inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
        if(m_useConcaveHull) {
            pcl::ConcaveHull<pcl::PointXYZRGB> chull;
            chull.setInputCloud (pc.makeShared());
            chull.setAlpha (m_concaveAlpha);
            chull.reconstruct (*cloud_hull);
        } else {
            pcl::ConvexHull<pcl::PointXYZRGB> chull;
            chull.setInputCloud (pc.makeShared());
            chull.reconstruct (*cloud_hull);
        }

        Eigen::Vector4f centroid, centroidAvg, tip, tipAvg;

        //calculate the centroid of the hull
        pcl::compute3DCentroid(*cloud_hull, centroid);

        //add the centroid to the moving average vector
        if(validatePointDistance(m_lastCentroid, centroid)) {
            this->addCentroidToAverage(centroid);
        } else {
            this->addCentroidToAverage(m_lastCentroid);
        }
        this->computeCentroidAverage(centroidAvg);

        //visualize starting point using a sphere
        pcl::PointXYZ o;
        o.x = centroidAvg(0);
        o.y = centroidAvg(1);
        o.z = centroidAvg(2);
        if(!vis->updateSphere(o, 0.01, 0.5, 0.5, 0.5, "centroid"))
            vis->addSphere(o, 0.01, "centroid", 0);

        //cut off points that are lying behind the centroid
        pcl::PassThrough<pcl::PointXYZRGB> passC;
        passC.setInputCloud (cloud_hull);
        passC.setFilterFieldName ("z");
        passC.setFilterLimits ((float)(o.z), (float)m_filterMax);
        passC.filter(*cloud_hull);

        //determine the point furthest from the center, should be the pointer tip
        pcl::getMaxDistance(*cloud_hull, centroid, tip);

        //add the centroid to the moving average vector
        if(validatePointDistance(m_lastTip, tip)) {
            this->addTipToAverage(tip);
        } else {
            this->addTipToAverage(m_lastTip);
        }
        this->computeTipAverage(tipAvg);

        //visualize tip point using a sphere
        o.x = tipAvg(0);
        o.y = tipAvg(1);
        o.z = tipAvg(2);
        if(!vis->updateSphere(o, 0.01, 0.5, 0.5, 0.5, "max"))
            vis->addSphere(o, 0.01, "max", 0);

        //publish the line based on the calculated start (center) and end (tip) points
        publishLine(centroidAvg, tipAvg);

        //copy the cloud to visualize in pclviewer
        m_pc = cloud_hull->makeShared();

        //update saved values
        m_lastCentroid = centroid;
        m_lastTip = tip;

    } else {
        //copy the original cloud to visualize in pclviewer
        m_pc = pc.makeShared();
    }

    //display the processed cloud
    this->displayCloud(*m_pc);
}

void kinproInteractor::displayCloud(pcl::PointCloud<pcl::PointXYZRGB> &pc, string id) {
//        if(!vis->updatePointCloud(pc, id)) {
//            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
//            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, rgb, id);
//            pclWidget->vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
////            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

//            cout << "Adding new RGB cloud" << endl;
//        }
        if(!vis->updatePointCloud<pcl::PointXYZRGB>(pc.makeShared(), id)) {      //adding <pcl::PointXYZRGB> leads to ignoring color values
            vis->addPointCloud<pcl::PointXYZRGB>(pc.makeShared(), id);

            cout<< "Adding new cloud: " << id << endl;
        }

}

void kinproInteractor::publishLine(Eigen::Vector4f start, Eigen::Vector4f end, float length) {

    Eigen::Vector4f newEnd, direction;
    direction = end - start;
    float l = sqrt(direction(0)*direction(0)+direction(1)*direction(1)+direction(2)*direction(2));
//    cout << "Distance: " << l << endl;


    geometry_msgs::Point s, e;
    if(l > m_lineLengthTresh) {

        newEnd = start + direction*length/l;

        s.x = start(0);
        s.y = start(1);
        s.z = start(2);

        e.x = newEnd(0);
        e.y = newEnd(1);
        e.z = newEnd(2);
    }else {
        s.x = s.y = s.z = 0;

        e.x = e.y = e.z = 0;
    }

    kinpro_interaction::line lineMsg;
    lineMsg.start = s;
    lineMsg.end = e;

    linePub.publish(lineMsg);
}


bool kinproInteractor::validatePointDistance(Eigen::Vector4f &pt1, Eigen::Vector4f &pt2) {
    Eigen::Vector4f diff = pt1-pt2;
    double dist = sqrt(diff(0)*diff(0)+diff(1)*diff(1)+diff(2)*diff(2));
    if(dist > m_maxPointDistance)
        return false;

    return true;
}

void kinproInteractor::addCentroidToAverage(Eigen::Vector4f &centroid) {
    if(this->pointerCentroids.size() < m_averageSize) {
        pointerCentroids.push_back(centroid);
    }else {
        pointerCentroids.erase(pointerCentroids.begin());
        pointerCentroids.push_back(centroid);
    }
}


void kinproInteractor::computeCentroidAverage(Eigen::Vector4f &average) {
    average = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0);
    for(size_t i = 0; i < pointerCentroids.size(); i++) {
        average += pointerCentroids.at(i);
    }
    average /= float(pointerCentroids.size());
}

void kinproInteractor::addTipToAverage(Eigen::Vector4f &centroid) {
    if(this->pointerTips.size() < m_averageSize) {
        pointerTips.push_back(centroid);
    }else {
        pointerTips.erase(pointerTips.begin());
        pointerTips.push_back(centroid);
    }
}


void kinproInteractor::computeTipAverage(Eigen::Vector4f &average) {
    average = Eigen::Vector4f(0.0, 0.0, 0.0, 0.0);
    for(size_t i = 0; i < pointerTips.size(); i++) {
        average += pointerTips.at(i);
    }
    average /= float(pointerTips.size());
}



/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag2pointcloud");

    kinproInteractor node;
    node.run();
    return 0;
}

//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>

//int user_data;

//void
//viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
//{
//    viewer.setBackgroundColor (1.0, 0.5, 1.0);
//    pcl::PointXYZ o;
//    o.x = 1.0;
//    o.y = 0;
//    o.z = 0;
//    viewer.addSphere (o, 0.25, "sphere", 0);
//    std::cout << "i only run once" << std::endl;

//}

//void
//viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
//{
//    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "Once per viewer loop: " << count++;
//    viewer.removeShape ("text", 0);
//    viewer.addText (ss.str(), 200, 300, "text", 0);

//    //FIXME: possible race condition here:
//    user_data++;
//}

//int
//main ()
//{
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//   // pcl::io::loadPCDFile ("my_point_cloud.pcd", *cloud);

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    //////////////////

//    //////////////////

//    //blocks until the cloud is actually rendered
//    viewer.showCloud(cloud);

//    //use the following functions to get access to the underlying more advanced/powerful
//    //PCLVisualizer

//    //This will only get called once
//    viewer.runOnVisualizationThreadOnce (viewerOneOff);


//    //This will get called once per visualization iteration
//    viewer.runOnVisualizationThread (viewerPsycho);
//    while (!viewer.wasStopped ())
//    {
//    //you can also do cool processing here
//    //FIXME: Note that this is running in a separate thread from viewerPsycho
//    //and you should guard against race conditions yourself...
//    user_data++;
//    }
//    return 0;
//}
