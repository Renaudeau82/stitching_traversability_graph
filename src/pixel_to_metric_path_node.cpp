/*-------------------------------------------------------------
 *
 *  This node use the /transform topic to change a path in pixels to its position in meters
 *
 * ----------------------------------------------------------------*/
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

class Worker
{
private:
    ros::NodeHandle nh; ///< Node Handle

    ros::Subscriber sub_path_; ///< Subscriber to stating point
    ros::Subscriber sub_transform_; ///< Subscriber to goal point

    ros::Publisher pTraj; ///< Publisher trajectory
    float minX;
    float scaleX;
    float minY;
    float scaleY;

public:
    Worker(); ///< Constructeur du worker
    void cbTransform(const std_msgs::Float32MultiArray &msg); ///< Callback tranform infos
    void cbPath(const std_msgs::Int32MultiArray &msg); ///< Callback for  the path to transform
};

Worker::Worker():nh("~")
{
    // subscribers
    sub_path_ = nh.subscribe("/trajectory_points",1,&Worker::cbPath,this);
    sub_transform_ = nh.subscribe("/transform",1,&Worker::cbTransform,this);
    // publishers
    pTraj = nh.advertise<std_msgs::Int32MultiArray>("/trajectory_metric_points",1);
    // variables
    minX = 0;
    scaleX=0;
    minY = 0;
    scaleY=0;
}

// Update variables
void Worker::cbTransform(const std_msgs::Float32MultiArray &msg)
{
    minX = msg.data[0];
    scaleX=msg.data[1];
    minY = msg.data[2];
    scaleY=msg.data[3];
}

// Transform the input path
void Worker::cbPath(const std_msgs::Int32MultiArray &msg)
{
    std_msgs::Float32MultiArray traj_msg;
    for(int i =0;i< msg.data.size(); i+=2)
    {
        float x = minX + scaleX*msg.data[i];
        float y = minY + scaleY*msg.data[i+1];
        traj_msg.data.push_back(x);
        traj_msg.data.push_back(y);
    }
    pTraj.publish(traj_msg);
}

int main(int argc, char ** argv)
{
    // Initialisation of ROS node
    ros::init(argc,argv,"pixel_to_metric_path");

    // call worker class
    Worker worker;

    ros::spin();
}
