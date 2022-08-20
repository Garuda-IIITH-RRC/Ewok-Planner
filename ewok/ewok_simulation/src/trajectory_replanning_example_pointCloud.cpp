/*** 
 * Trajectory Replanning example using EWOK using point cloud data
 * Code Written by -: Deepak Singh
***/




#include <thread>
#include <chrono>
#include <map>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <visualization_msgs/MarkerArray.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>

#include<sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

double dt ;
int num_opt_points;

bool initialized = false;
Eigen::Vector3d currPose;
float last_ppx = 0.0, last_ppy = 0.0, last_ppz = 0.0;
geometry_msgs::Point pp;

// Eigen::Vector3d(start_x, start_y, start_z)

tf::Quaternion RQ2;  
double currroll,currpitch,curryaw;


std::ofstream f_time, opt_time;


ewok::PolynomialTrajectory3D<10>::Ptr traj;
ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;
ewok::UniformBSpline3DOptimization<POW>::Ptr spline_optimization;
//ewok::UniformBSpline3DOptimization<7>::Ptr spline_optimization;

ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, current_traj_pub;
tf::TransformListener * listener;

void pointCloudCallback(const sensor_msgs::PointCloud2 msg)
{
    // ROS_INFO("received point cloud");

    tf::StampedTransform transform;

    try{

        listener->lookupTransform("map", "camera_d455_link", msg.header.stamp, transform);        
    }

    catch (tf::TransformException &ex){
        ROS_INFO("Couldn't get transform");
        ROS_WARN("%s", ex.what());
    }

    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    auto t1 = std::chrono::high_resolution_clock::now();


    // convert this pointcloud to ewok::pointCloud
    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;

    Eigen::Vector4f p;

    for (sensor_msgs::PointCloud2ConstIterator<float> it(msg, "x"); it != it.end(); ++it)
    {
        p(0) = float(it[0]);
        p(1) = float(it[1]);
        p(2) = float(it[2]);
        p(3) = 1;

        // apply the transform on the points
        p = T_w_c*p;

        cloud1.push_back(p);
    }

    Eigen::Vector3f origin = (T_w_c *  Eigen::Vector4f(0,0,0,1)).head<3>();

    auto t2 = std::chrono::high_resolution_clock::now();


    if(!initialized) {
        Eigen::Vector3i idx;
        edrb->getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        edrb->setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);

        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            edrb->moveVolume(diff);

            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }

    }

    //ROS_INFO_STREAM("cloud1 size: " << cloud1.size());


    auto t3 = std::chrono::high_resolution_clock::now();

    edrb->insertPointCloud(cloud1, origin);

    auto t4 = std::chrono::high_resolution_clock::now();

    f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

    visualization_msgs::Marker m_occ, m_free;
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);


    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);

}

void sendCommandCallback(const ros::TimerEvent& e) {

    float euclidian_distance = sqrt( pow( (last_ppx - currPose(0)), 2) +
    pow ( (last_ppy - currPose(1)), 2) + 
    pow ((last_ppz - currPose(2)), 2)  ) ;


    auto t1 = std::chrono::high_resolution_clock::now();

    edrb->updateDistance();

    visualization_msgs::MarkerArray traj_marker;

    auto t2 = std::chrono::high_resolution_clock::now();
    spline_optimization->optimize();
    auto t3 = std::chrono::high_resolution_clock::now();

    opt_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " "
        << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << std::endl;

    Eigen::Vector3d pc = spline_optimization->getFirstOptimizationPoint();
    pp.x = pc[0];
    pp.y = pc[1];
    pp.z = pc[2];
    // trajectory_pub.publish(pp);
    spline_optimization->getMarkers(traj_marker);
    current_traj_pub.publish(traj_marker);

    visualization_msgs::Marker m_dist;
    edrb->getMarkerDistance(m_dist, 0.5);
    dist_marker_pub.publish(m_dist);

    last_ppx = pp.x;
    last_ppy = pp.y;
    last_ppz = pp.z;

    if (euclidian_distance < 1.5){
    spline_optimization->addLastControlPoint();
    }

    else{
    ROS_INFO_STREAM(" EUCLDIAN WARNING :" << euclidian_distance);
    }
    trajectory_pub.publish(pp);

    // ROS_INFO_STREAM("Waypoint Callback");
}


void local_pose_cb(const geometry_msgs::PoseStamped pose)
{
    currPose(0) = pose.pose.position.x;
    currPose(1) = pose.pose.position.y;
    currPose(2) = pose.pose.position.z;

    tf::quaternionMsgToTF(pose.pose.orientation,RQ2); 
    tf::Matrix3x3(RQ2).getRPY(currroll,currpitch,curryaw);

    // ROS_INFO_STREAM(curryaw);

    // start_x = currPose(0);
    // start_y = currPose(1);
    // start_z = currPose(2);
    // start_yaw = curryaw;

}


int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_replanning_example");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string path = ros::package::getPath("ewok_simulation") + "/benchmarking/";

    ROS_INFO_STREAM("path: " << path);

    f_time.open(path + "mapping_time.txt");
    opt_time.open(path + "optimization_time.txt");


    listener = new tf::TransformListener;

    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);

    trajectory_pub =
            nh.advertise<geometry_msgs::Point>(
                    "command/point", 10);

    ros::Publisher traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);

    current_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1, true);

    // ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,pointCloudCallback);

    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_image_sub_ ;
    depth_image_sub_.subscribe(nh, "/camera_d455/depth/color/points", 5);
    std::cout<<"here2"<<std::endl;
    // depth_image_sub_.subscribe(nh, "output", 5);
    

    tf::MessageFilter<sensor_msgs::PointCloud2> tf_filter_(depth_image_sub_, *listener, "map", 5);
    tf_filter_.registerCallback(pointCloudCallback);

    ros::Subscriber location = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,local_pose_cb);


    double max_velocity, max_acceleration;
    pnh.param("max_velocity", max_velocity, 0.75);
    pnh.param("max_acceleration", max_acceleration, 1.0);

    Eigen::Vector4d limits(max_velocity, max_acceleration, 0, 0);


    double resolution;
    pnh.param("resolution", resolution, 0.13);
    edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));

    double distance_threshold;
    pnh.param("distance_threshold", distance_threshold, 1.0);

    double start_x, start_y, start_z, start_yaw;
    pnh.param("start_x", start_x, 0.0);
    pnh.param("start_y", start_y, 0.0);
    pnh.param("start_z", start_z, 0.0);
    pnh.param("start_yaw", start_yaw, 0.0);

    double middle_x, middle_y, middle_z, middle_yaw;
    pnh.param("middle_x", middle_x, 0.0);
    pnh.param("middle_y", middle_y, 0.0);
    pnh.param("middle_z", middle_z, 0.0);
    pnh.param("middle_yaw", middle_yaw, 0.0);

    double stop_x, stop_y, stop_z, stop_yaw;
    pnh.param("stop_x", stop_x, 0.0);
    pnh.param("stop_y", stop_y, 0.0);
    pnh.param("stop_z", stop_z, 0.0);
    pnh.param("stop_yaw", stop_yaw, 0.0);


    pnh.param("dt", dt, 0.5);
    pnh.param("num_opt_points", num_opt_points, 7);

    // ROS_INFO("Started hovering example with parameters: start - %f %f %f %f, middle - %f %f %f %f, stop - %f %f %f %f",
    //          start_x, start_y, start_z, start_yaw,
    //          middle_x, middle_y, middle_z, middle_yaw,
    //          stop_x, stop_y, stop_z, stop_yaw);

    // ROS_INFO("dt: %f, num_opt_points: %d", dt, num_opt_points);


    ewok::Polynomial3DOptimization<10> to(limits*0.6);


    {
        typename ewok::Polynomial3DOptimization<10>::Vector3Array path;

        path.push_back(Eigen::Vector3d(start_x, start_y, start_z));
        path.push_back(Eigen::Vector3d(middle_x, middle_y, middle_z));
        path.push_back(Eigen::Vector3d(stop_x, stop_y, stop_z));

        traj = to.computeTrajectory(path);

        visualization_msgs::MarkerArray traj_marker;
        traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
        traj_marker_pub.publish(traj_marker);

    }

    spline_optimization.reset(new ewok::UniformBSpline3DOptimization<POW>(traj, dt));
    //spline_optimization.reset(new ewok::UniformBSpline3DOptimization<7>(traj, dt));

    for (int i = 0; i < num_opt_points; i++) {
        spline_optimization->addControlPoint(Eigen::Vector3d(start_x, start_y, start_z));
    }

    spline_optimization->setNumControlPointsOptimized(num_opt_points);
    spline_optimization->setDistanceBuffer(edrb);
    spline_optimization->setDistanceThreshold(distance_threshold);
    spline_optimization->setLimits(limits);


    geometry_msgs::Point p;
    p.x = start_x;
    p.y = start_y;
    p.z = start_z;

    //Eigen::Vector3d desired_position(start_x, start_y, start_z);
    double desired_yaw = start_yaw;

    // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f, %f].",
    //          nh.getNamespace().c_str(),
    //          start_x,
    //          start_y,
    //          start_z, start_yaw);

    trajectory_pub.publish(p);

    ros::Duration(1.0).sleep();

    ros::Timer timer = nh.createTimer(ros::Duration(dt), sendCommandCallback);

    ros::spin();

    f_time.close();
    opt_time.close();
}


// FOR MOST SAFEST FLIGHT 
// 1. REDUCE     
//     pnh.param("max_velocity", max_velocity, 1.0);
//     pnh.param("max_acceleration", max_acceleration, 1.0);
// ROBOT WILL MOVE SLOW BUT SAFEST

// 2. INCREASE 
//         pnh.param("distance_threshold", distance_threshold, 1.0);

// 3. KEEP dT 0.5 TO 0.8
// MORE dt IT WILL SOLVE SLOW
// LESS dt COMPUTATION COST VERY HEAVY

// 4. KEEP num_opt_points 5 TO 10

// 5. DECERASE  euclidian_distance; KEEP 1 TO 2
// If (euclidian_distance < 1.5)
