#include <kinpro_interaction/kinpro_interaction.h>

using namespace std;

kinproInteractor::kinproInteractor() {

    this->nh        = new ros::NodeHandle;

    nh->param<std::string>("cloud_in",   pointCloudTopic,    "/camera/depth_registered/points");
    nh->param<double>("kinpro_interaction/filter_min", filterMin, 0.0);
    nh->param<double>("kinpro_interaction/filter_max", filterMax, 0.3);
    nh->param<bool>("kinpro_interaction/use_concave_hull", useConcaveHull, true);
    nh->param<double>("kinpro_interaction/concave_alpha", concaveAlpha, 0.01);
    nh->param<double>("kinpro_interaction/plane_dist_thresh", planeDistThresh, 0.3);



    pc_sub   = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGB> >(pointCloudTopic, 2, &kinproInteractor::pointcloudCallback, this);

    linePub = nh->advertise< kinpro_interaction::line >("/line", 1, this);

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


    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (msg->makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ((float)filterMin, (float)filterMax);
    //pass.setFilterLimitsNegative (true);
    pass.filter(pc);
//    this->displayCloud(pc, "msg");

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    if(!pc.points.empty()) {
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (planeDistThresh);

        seg.setInputCloud (pc.makeShared());
        seg.segment (*inliers, *coefficients);
    }

    if(!inliers->indices.empty()) {

        // Project the model inliers
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setIndices (inliers);
        proj.setInputCloud (pc.makeShared());
        proj.setModelCoefficients (coefficients);
        proj.filter (pc);

        //    //extract the model inliers
        //    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        //    extract.setInputCloud (pc.makeShared());
        //    extract.setIndices (inliers);
        //    extract.setNegative (false);
        //    extract.filter (pc);

        // Create a Concave Hull representation of the projected inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
        if(useConcaveHull) {
            pcl::ConcaveHull<pcl::PointXYZRGB> chull;
            chull.setInputCloud (pc.makeShared());
            chull.setAlpha (concaveAlpha);
            chull.reconstruct (*cloud_hull);
        } else {
            pcl::ConvexHull<pcl::PointXYZRGB> chull;
            chull.setInputCloud (pc.makeShared());
            chull.reconstruct (*cloud_hull);
        }



        Eigen::Vector4f centroid, maxPt;
        pcl::PointXYZ o;

        pcl::compute3DCentroid(*cloud_hull, centroid);
        o.x = centroid(0);
        o.y = centroid(1);
        o.z = centroid(2);
        if(!vis->updateSphere(o, 0.01, 0.5, 0.5, 0.5, "centroid"))
            vis->addSphere(o, 0.01, "centroid", 0);


        pcl::PassThrough<pcl::PointXYZRGB> passC;
        passC.setInputCloud (cloud_hull);
        passC.setFilterFieldName ("z");
        passC.setFilterLimits ((float)(o.z), (float)filterMax);
        passC.filter(*cloud_hull);

        pcl::getMaxDistance(*cloud_hull, centroid, maxPt);
        o.x = maxPt(0);
        o.y = maxPt(1);
        o.z = maxPt(2);
        if(!vis->updateSphere(o, 0.01, 0.5, 0.5, 0.5, "max"))
            vis->addSphere(o, 0.01, "max", 0);
        //    for( pcl::PointCloud<pcl::PointXYZRGB>::iterator iter = cloud_hull->begin(); iter != cloud_hull->end(); ++iter)
        //    {
        //        pcl::PointXYZ o;
        //        o.x = iter->x;
        //        o.y = iter->y;
        //        o.z = iter->z;
        //        if(!vis->updateSphere( o, 0.01, 0.5, 0.5, 0.5, "sphere"))
        //                vis->addSphere(o, 0.01, "sphere", 0);
        //    }

        publishLine(centroid, maxPt);

        m_pc = cloud_hull->makeShared();
    } else {
        m_pc = pc.makeShared();
    }
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
    newEnd = start + direction*length/l;

    geometry_msgs::Point s, e;
    s.x = start(0);
    s.y = start(1);
    s.z = start(2);

    e.x = newEnd(0);
    e.y = newEnd(1);
    e.z = newEnd(2);

    kinpro_interaction::line lineMsg;
    lineMsg.start = s;
    lineMsg.end = e;

    linePub.publish(lineMsg);
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
