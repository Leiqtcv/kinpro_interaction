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
    nh->param<double>("kinpro_interaction/sphere_radius", m_sphereRadius, 0.01);
    nh->param<bool>("kinpro_interaction/visualize", m_visualize, false);
    nh->param<bool>("kinpro_interaction/debug", m_debug, false);
    nh->param<int>("kinpro_interaction/debug_nr", m_debug_nr, 0);


    pc_sub   = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGB> >(pointCloudTopic, 2, &kinproInteractor::pointcloudCallback, this);

    linePub = nh->advertise< kinpro_interaction::line >("/line", 1, this);

    pauseVisOdomClient = nh->serviceClient < std_srvs::Empty >("/pause_fovis");
    resumeVisOdomClient = nh->serviceClient < std_srvs::Empty >("/resume_fovis");

    m_lastCentroid = Eigen::Vector4f(0,0,0,1);
    m_lastTip = Eigen::Vector4f(0,0,0,1);

    if(m_visualize){
        vis = new pcl::visualization::PCLVisualizer("vis", true);
        vis->setBackgroundColor(0.2, 0.2, 0.2);
        vis->addCoordinateSystem(0.05, 0.0, 0.0, 0.3);
    }
}

kinproInteractor::~kinproInteractor() {

}

void kinproInteractor::run() {
    ros::Rate rate(30);

    if(m_visualize) {
        while (!vis->wasStopped() && ros::ok()){
            ros::spinOnce();
            vis->spinOnce();
            rate.sleep();
        }
    }else {
        while (ros::ok()){
           ros::spinOnce();
           rate.sleep();
        }
    }
    cout << "stopped!" << endl;
}

void kinproInteractor::pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg) {

    Eigen::Vector4f centroid(0,0,0,1), centroidAvg(0,0,0,1), tip(0,0,0,1), tipAvg(0,0,0,1);

    //IMAGES
    if(m_visualize)
        vis->removeAllShapes();

    //IMAGES
    if(m_debug && m_debug_nr == 0)
        m_pc = msg->makeShared();


    //filter the input cloud to contain only the values inside of reach of the user
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (msg->makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ((float)m_filterMin, (float)m_filterMax);
    pass.filter(pc);

    //IMAGES
    if(m_debug && m_debug_nr >= 1)
        m_pc = pc.makeShared();

    //display the filtered cloud
//    this->displayCloud(pc, "msg");

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    //segmentate the plane of the pointing device to speed up next calculations
    if(!pc.points.empty()) {
        pauseVisOdomClient.call(m_e);
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

    }else {
        resumeVisOdomClient.call(m_e);
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

        //IMAGES
        if(m_debug && m_debug_nr >= 2)
            m_pc = pc.makeShared();

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

        //IMAGES
        if(m_debug && m_debug_nr >= 3)
            m_pc = cloud_hull->makeShared();

        //calculate the centroid of the hull
        pcl::compute3DCentroid(*cloud_hull, centroid);

        //add the centroid to the moving average vector
        if(validatePointDistance(m_lastCentroid, centroid)) {
            this->addCentroidToAverage(centroid);
        } else {
            this->addCentroidToAverage(m_lastCentroid);
            m_resetCounter++;
        }
        this->computeCentroidAverage(centroidAvg);

        //visualize starting point using a sphere
        pcl::PointXYZ o,p;
        o.x = centroidAvg(0);
        o.y = centroidAvg(1);
        o.z = centroidAvg(2);
        if(m_visualize || (m_debug && m_debug_nr >= 4)) {
            if(!vis->updateSphere(o, m_sphereRadius, 1.0, 0.0, 0.0, "centroid"))
                vis->addSphere(o, m_sphereRadius, "centroid", 0);
        }

        //cut off points that are lying before the centroid
        pcl::PassThrough<pcl::PointXYZRGB> passC;
        passC.setInputCloud (cloud_hull);
        passC.setFilterFieldName ("z");
        passC.setFilterLimits ((float)(o.z), (float)m_filterMax);
        passC.filter(*cloud_hull);

        //IMAGES
        if(m_debug && m_debug_nr >= 5)
            m_pc = cloud_hull->makeShared();

        //determine the point furthest from the center, should be the pointer tip
        pcl::getMaxDistance(*cloud_hull, centroid, tip);

        //add the centroid to the moving average vector
        if(validatePointDistance(m_lastTip, tip)) {
            this->addTipToAverage(tip);
        } else {
            this->addTipToAverage(m_lastTip);
            m_resetCounter++;
        }
        this->computeTipAverage(tipAvg);

        //visualize tip point using a sphere
        p.x = tipAvg(0);
        p.y = tipAvg(1);
        p.z = tipAvg(2);
        if(m_visualize || (m_debug && m_debug_nr >= 6)) {
            if(!vis->updateSphere(p, m_sphereRadius, 1.0, 0.0, 0.0, "max"))
                vis->addSphere(p, m_sphereRadius, "max", 0);
        }

        //IMAGES
        if(m_debug && m_debug_nr >= 7){
            p.x = p.x + 2.0*(p.x - o.x);
            p.y = p.y + 2.0*(p.y - o.y);
            p.z = p.z + 2.0*(p.z - o.z);
            vis->addLine(o,p,1.0,1.0,1.0,"line");
        }

        //copy the cloud to visualize in pclviewer
        if(m_visualize && !m_debug){
            m_pc = cloud_hull->makeShared();
        }

        //update saved values
        if(m_resetCounter > 30){
            pointerCentroids.clear();
            pointerTips.clear();
            m_lastCentroid = centroid;
            m_lastTip = tip;
            m_resetCounter = 0;
        }else{
            m_lastCentroid = centroidAvg;
            m_lastTip = tipAvg;
        }

    } else {
        //copy the original cloud to visualize in pclviewer
        m_pc = pc.makeShared();
    }

    //publish the line based on the calculated start (center) and end (tip) points
    publishLine(centroidAvg, tipAvg);

    //display the processed cloud
    if(m_visualize) {
        this->displayCloud(*m_pc);
    }
}

void kinproInteractor::displayCloud(pcl::PointCloud<pcl::PointXYZRGB> &pc, string id) {
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
