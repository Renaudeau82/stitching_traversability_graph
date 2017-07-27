/*--
 *      This node do the stitching of elevation map and compute the traersability map from it
 *
 *      Subscrib :
 *          /TF to retreave sensor pose
 *          /stereo_point2 point cloud from stereo reconstruction
 *
 *      Publish :
 *          /image_traversability the binary image of Traversability
 *          /pt_source the point to start the path
 *          /pt_dest the goal for the path
 *          /transform (xmin, scalex, ymin, scaley) to go back into metric space
 *
 *      Service :
 *          /publish_srv to ask for the traversability if the auto_pub param is not true
 *
 *      Parameters :
 *          verbose to show steps, time consuming, image processed
 *          automatic_pub if false you have to call the servicce to publish datas
 *          resolution resolution of the image (pixels/m)
 *          gamma falue of the parameter for gamma correction
 *          robot_size the size of the robot for filtering and path sampling
 *
 *      Approach :
 *          1) projection the local point cloud into a elevation image (outlier are detected by knowing the limits of the area before !!!)
 *          2) stiching this local image in the full image using kernel approach to give less weight to border points
 *          3) compute normals of the elevation image (gradiant + orientation)
 *          4) compute traversability using segmentation on normal_z
 *          5) filtering traversability images *
 */
#include <stitching_traversability_graph.hpp>

TraversabilityGraphStitcher::TraversabilityGraphStitcher(ros::NodeHandle nh,
                                                         ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(kDefaultVerbose),
      resolution_(kDefaultResolution),
      gamma_(kDefaultGamma),
      robot_size_(kDefaultRobotSize),
      automatic_pub_(kDefaultAutomaticPub),
      world_frame_("enu")
{
    // Initial interaction with ROS
    getParametersFromRos();
    subscribeToTopics();
    advertiseTopics();
    advertiseServices();

    // initialise variables
    first = true;
    zscale = 255;
}

void TraversabilityGraphStitcher::subscribeToTopics() {
    pcd_sub_ = nh_.subscribe("/pcd", 10, &TraversabilityGraphStitcher::pointCloudCallback, this);
}

void TraversabilityGraphStitcher::advertiseTopics() {
    image_transport::ImageTransport it(nh_);
    img_pub_ = it.advertise("/image_traversability", 1);
    source_pub_ = nh_.advertise<geometry_msgs::Point>("/pt_source",1);
    dest_pub_ = nh_.advertise<geometry_msgs::Point>("/pt_dest",1);
    transform_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/transform", 1, true);
}

void TraversabilityGraphStitcher::advertiseServices() {
    publish_srv_  = nh_private_.advertiseService("publish_srv", &TraversabilityGraphStitcher::publishCallback, this);
}

void TraversabilityGraphStitcher::getParametersFromRos() {
    nh_private_.param("verbose", verbose_, verbose_);
    nh_private_.param("automatic_pub", automatic_pub_, automatic_pub_);
    nh_private_.param("resolution", resolution_, resolution_);
    nh_private_.param("gamma", gamma_, gamma_);
    nh_private_.param("robot_size", robot_size_, robot_size_);
}

bool TraversabilityGraphStitcher::publishCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    pub = true;
}

void TraversabilityGraphStitcher::pointCloudCallback(const sensor_msgs::PointCloud2::Ptr& pcd_msg)
{
    if (verbose_) ROS_INFO_STREAM("PointCloud received, starting treatment.    ");
    ros::Time time0, time1, time2;
    double duration;
    time0 = ros::Time::now();

    /// Load input PointCloud2 into a PointCloud<T>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*pcd_msg, *cloud);

    /// Look up transform from sensor frame to world frame.
    voxblox::Transformation T_G_C;
    if (lookupTransformTf(pcd_msg->header.frame_id, world_frame_, pcd_msg->header.stamp, &T_G_C))
    {
        for (size_t pt_idx = 0; pt_idx < cloud->size(); ++pt_idx)
        {
            Eigen::Matrix<double, 3, 1>  point_C;
            point_C(0,0) = cloud->points[pt_idx].x;
            point_C(1,0) = cloud->points[pt_idx].y;
            point_C(2,0) = cloud->points[pt_idx].z;
            Eigen::Matrix<double, 3, 1>  point_G = T_G_C * point_C;
            cloud->points[pt_idx].x = point_G(0,0);
            cloud->points[pt_idx].y = point_G(1,0);
            cloud->points[pt_idx].z = point_G(2,0);
        }
    }
    else return;

    /// clean outliers ( 2.5<x<21 && 25<y<45 && -0.6<z) // zone position, limits rosbag1!!!
    /// clean outliers ( -7<x<26 && -4<y<27 && -0.6<z) // zone position, limits rosbag2!!!
    pcl::PointIndices::Ptr indices_in (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> eifilter (false);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZ point = cloud->points[i];
        //if(point.data[0]>2.5 && point.data[0]< 21 && point.data[1]>25 && point.data[1]<45 && point.data[2]>-0.6){
        if(point.data[0]>-7 && point.data[0]< 26 && point.data[1]>-4 && point.data[1]<27 && point.data[2]>-0.6){
            indices_in->indices.push_back(i);
        }
    }
    eifilter.setIndices(indices_in);
    eifilter.filterDirectly(cloud);
    // Remove Nan in pointCloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);

    //--------------------- traitement du nuage de point ---------------------------------------------------//
    /// Make pcd into depth image if enougth points
    if(cloud->points.size() > 600)
    {
        if (verbose_) ROS_INFO_STREAM("Compution of pcd_image : "<<cloud->points.size()<<" points    ");
        time1 = ros::Time::now();
        // find limits of pcd
        double xmin = 1000;
        double xmax =-1000;
        double ymin = 1000;
        double ymax =-1000;
        double zmin = 1000;
        double zmax =-1000;
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            if( point.data[0] < xmin) xmin = point.data[0];
            if( point.data[0] > xmax) xmax = point.data[0];
            if( point.data[1] < ymin) ymin = point.data[1];
            if( point.data[1] > ymax) ymax = point.data[1];
            if( point.data[2] < zmin) zmin = point.data[2];
            if( point.data[2] > zmax) zmax = point.data[2];
        }
        // create tmp images
        int width = resolution_*(xmax-xmin)+1;
        int height = resolution_*(ymax-ymin)+1;
        double zscale_tmp = std::min(255/(zmax+0.6),zscale); // choose the optimal scale
        if (verbose_) ROS_INFO_STREAM("area : "<<xmin<<"-"<<xmax<<" , "<<ymin<<"-"<<ymax<<" , "<<zmin<<"-"<<zmax<<" -> scale="<<resolution_<<" ("<<width<<","<<height<<") zscale="<<zscale_tmp);
        if(width < 0 || height < 0) return;
        cv::Mat elevation_tmp = cv::Mat::zeros(height, width, CV_8UC1);
        cv::Mat pcd_in_tmp = cv::Mat::zeros(height, width, CV_8UC1);
        // projection into elevation image
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            unsigned int y = (unsigned int)(resolution_*(point.data[1]-ymin));
            unsigned int x = (unsigned int)(resolution_*(point.data[0]-xmin));
            if(y<0 || y>height-1) ROS_ERROR_STREAM("erreur en y : "<<y);
            if(x<0 || x>width-1) ROS_ERROR_STREAM("erreur en x : "<<x);
            elevation_tmp.at<unsigned char>(y,x) = (unsigned char)(zscale_tmp*(cloud->points[i].data[2] + 0.6)); // +0.6 = -zmin
            pcd_in_tmp.at<unsigned char>(y,x) = 255;
        }
        time2 = ros::Time::now();
        duration = time2.toSec() - time1.toSec();
        if(verbose_) ROS_INFO_STREAM(duration<<"sec                       ");
        //if (verbose_) cv::imshow("elevation_tmp",elevation_tmp);

        /// Initialisation of global image with the first image
        if(first)
        {
            minX = xmin;
            maxX = xmax;
            minY = ymin;
            maxY = ymax;
            zscale = zscale_tmp;
            elevation_full_image = elevation_tmp;
            pcd_in_full_image = pcd_in_tmp;
            first = false;
        }
        /// stitching of elevation_tmp on globale image
        else
        {
            if (verbose_) ROS_INFO("Stitching");
            time1 = ros::Time::now();
            double resizeScale =1;
            if( zmax*zscale > 255) // we have tu change the z scale to fit the 8U image
            {
                resizeScale = zscale_tmp / zscale;
                zscale = zscale_tmp;
            }
            // define the new full image
            int width = resolution_*(std::max(xmax,maxX)-std::min(xmin,minX)) +1;
            int height = resolution_*(std::max(ymax,maxY)-std::min(ymin,minY)) +1;
            cv::Mat elevation_new = cv::Mat::zeros(height, width, CV_8UC1);
            cv::Mat pcd_in_new = cv::Mat::zeros(height, width, CV_8UC1);
            // copy old elevation_full
            for(unsigned int i=0;i<elevation_full_image.rows;i++)
            {
                for(unsigned int j=0;j<elevation_full_image.cols;j++)
                {
                    unsigned int I = (unsigned int) i+std::max(0.0,(minY-ymin)*resolution_);
                    unsigned int J = (unsigned int) j+std::max(0.0,(minX-xmin)*resolution_);
                    if(I<0 || I>height-1) ROS_ERROR_STREAM("erreur en i : "<<I);
                    if(J<0 || J> width-1) ROS_ERROR_STREAM("erreur en j : "<<J);
                    elevation_new.at<unsigned char>(I,J) = elevation_full_image.at<unsigned char>(i,j) * resizeScale;
                    pcd_in_new.at<unsigned char>(I,J) = pcd_in_full_image.at<unsigned char>(i,j);
                }
            }
            // Update !  kernel merging of elevation_map
            double kernel_coef = std::min(1.0/elevation_tmp.cols,1.0/elevation_tmp.rows);
            if(verbose_) ROS_INFO_STREAM("kernel_coef = "<<kernel_coef);
            for(unsigned int i=0;i<elevation_tmp.rows;i++)
            {
                for(unsigned int j=0;j<elevation_tmp.cols;j++)
                {
                    unsigned int I = (unsigned int) i+std::max(0.0,(ymin-minY)*resolution_);
                    unsigned int J = (unsigned int) j+std::max(0.0,(xmin-minX)*resolution_);
                    if(I<0 || I>height-1) ROS_ERROR_STREAM("erreur en i : "<<I);
                    if(J<0 || J> width-1) ROS_ERROR_STREAM("erreur en j : "<<J);
                    if(elevation_tmp.at<unsigned char>(i,j) > 1)
                    {
                        if(elevation_full_image.at<unsigned char>(I,J) > 1) // update of an image point
                        {
                            // weigth change with the distance to the center of the image
                            double dist = std::sqrt((elevation_tmp.cols/2.0 - j)*(elevation_tmp.cols/2.0 - j) + (elevation_tmp.rows/2.0 - i)*(elevation_tmp.rows/2.0 - i));
                            double alpha = std::min(std::max(dist*kernel_coef,0.0),0.5) + 0.5;
                            elevation_new.at<unsigned char>(I,J) = alpha*elevation_new.at<unsigned char>(I,J) + (1-alpha)*(elevation_tmp.at<unsigned char>(i,j)); // + 40 offset for visualisation
                        }
                        else // new point in image
                        {
                            elevation_new.at<unsigned char>(I,J) = elevation_tmp.at<unsigned char>(i,j);
                        }
                        pcd_in_new.at<unsigned char>(I,J) = pcd_in_tmp.at<unsigned char>(i,j);
                    }
                }
            }
            // actualization
            elevation_full_image = elevation_new;
            pcd_in_full_image = pcd_in_new;
            if( xmin < minX)
                minX = xmin;
            if( xmax > maxX)
                maxX = xmax;
            if( ymin < minY)
                minY = ymin;
            if( ymax > maxY)
                maxY = ymax;
            time2 = ros::Time::now();
            duration = time2.toSec() - time1.toSec();
            if(verbose_) ROS_INFO_STREAM(duration<<"sec");
            if (verbose_) ROS_INFO_STREAM("zone : "<<minX<<"-"<<maxX<<" , "<<minY<<"-"<<maxY<<" -> scale="<<resolution_<<" ("<<width<<","<<height<<") zscale="<<zscale);
        }

        if (verbose_) cv::imshow("elevation_full",elevation_full_image);

        // if the traversability_image is needed
        if(automatic_pub_ || pub)
        {
            /// Using Opencv to treat elevation map
            if (verbose_) ROS_INFO("Compution normals                   ");
            time1 = ros::Time::now();
            cv::Mat modifiedImage = elevation_full_image.clone();
            gammaCorrection(modifiedImage, modifiedImage, gamma_);// cheating

            // gradiant
            int ddepth = CV_32F;
            cv::Mat grad_x, grad_y;
            cv::Scharr( modifiedImage, grad_x, ddepth, 1, 0, 1, 0, cv::BORDER_DEFAULT );
            cv::Scharr( modifiedImage, grad_y, ddepth, 0, 1, 1, 0, cv::BORDER_DEFAULT );
            cv::Mat gradiant = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F);
            cv::magnitude(grad_x,grad_y,gradiant);
            // filtering gaussien to reduce higth frequency effect like small pics
            cv::GaussianBlur(gradiant,gradiant,cv::Size(3,3),0,0);

            // orientation
            cv::Mat orientation = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F); //to store the gradients grad_x.convertTo(grad_x,CV_32F);
            cv::phase(grad_x, grad_y, orientation,true);
            cv::normalize(orientation, orientation, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);

            // computing normal_z for the elevation
            cv::Mat normal_z = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F);
            double coef = resolution_  * zscale / 16; // 16 for half the sum of scharr pattern
            for (int i=0; i < modifiedImage.rows;i++)
            {
                for (int j=0; j < modifiedImage.cols;j++)
                {
                    if(pcd_in_full_image.at<uchar>(i,j) != 0)
                    {
                        double angle = std::atan(gradiant.at<float>(i,j)/coef);
                        Eigen::Vector3d normal(sin(angle) * sin(orientation.at<uchar>(i,j)*2*M_PI/255.0) , sin(angle) * cos(orientation.at<uchar>(i,j)*2*M_PI/255.0) , cos(angle));
                        normal.normalize();
                        normal_z.at<float>(i,j) = (float)normal.z();
                    }
                }
            }
            time2 = ros::Time::now();
            duration = time2.toSec() - time1.toSec();
            if(verbose_) ROS_INFO_STREAM(duration<<"sec                 ");

            /// Computation of image traversability
            if (verbose_) ROS_INFO("Compution of traversability             ");
            time1 = ros::Time::now();
            traversability_full_image = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_8U);
            // segmentation of traversability (thresholding)
            for (int i=0; i < modifiedImage.rows;i++)
            {
                for (int j=0; j < modifiedImage.cols;j++)
                {
                    if(elevation_full_image.at<uchar>(i,j) < 250 && pcd_in_full_image.at<uchar>(i,j) > 0  && normal_z.at<float>(i,j) > 0.80)
                        traversability_full_image.at<uchar>(i,modifiedImage.cols-j+1) = 255; // flip image to be z downward
                }
            }
            //if (verbose_) cv::imshow("traversability_image_raw",traversability_full_image);
            // median filter to remove small black pixels
            cv::medianBlur(traversability_full_image,traversability_full_image,5);
            // erode to make bigger the places to evode depanding on the robot_size
            int element_size = (int)robot_size_/4;
            cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*element_size + 1, 2*element_size+1 ), cv::Point( element_size, element_size ) );
            cv::morphologyEx( traversability_full_image, traversability_full_image, cv::MORPH_ERODE, element );
            time2 = ros::Time::now();
            duration = time2.toSec() - time1.toSec();
            if (verbose_) ROS_INFO_STREAM(duration<<"sec            ");
            //cv::imshow("traversability",traversability_full_image);

            /// colorfull drawing
            cv::normalize(gradiant, gradiant,0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
            int amplify_magnitude = 20;// amplfy gradiant for visualisation
            for(unsigned int i=0;i<grad_x.rows;i++)
            {
                for(unsigned int j=0;j<grad_y.cols;j++)
                {
                    if(gradiant.at<uchar>(i,j)*amplify_magnitude < 255)
                        gradiant.at<uchar>(i,j) = gradiant.at<uchar>(i,j)*amplify_magnitude;
                    else
                        gradiant.at<uchar>(i,j) = 255;
                }
            }
            cv::Mat fusion = cv::Mat::zeros(grad_x.rows, grad_y.cols, CV_8UC3);
            cv::cvtColor(fusion,fusion,cv::COLOR_BGR2HSV);
            for(unsigned int i=0;i<grad_x.rows;i++)
            {
                for(unsigned int j=0;j<grad_y.cols;j++)
                {
                    fusion.at<cv::Vec3b>(i,j) = cv::Vec3b(orientation.at<unsigned char>(i,j),200,(gradiant.at<unsigned char>(i,j)));
                }
            }
            cv::cvtColor(fusion,fusion,cv::COLOR_HSV2BGR);
            //if(verbose_) cv::imshow("fusion", fusion);
            //cv::imshow("gradiant",gradiant);
            //cv::imshow("orientation",orientation);
            cv::normalize(normal_z, normal_z, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
            cv::imshow("normal_z",normal_z);

            //--------------------- Publishing ---------------------------------------------------//
            if (verbose_) ROS_INFO("Publishing ");
            time1 = ros::Time::now();

            /// publishing points
            geometry_msgs::Point source; // first point
            for (int i=0; i < traversability_full_image.rows*traversability_full_image.cols;i++)
            {
                if(traversability_full_image.at<uchar>(i/traversability_full_image.cols,i%traversability_full_image.cols) > 100)
                {
                    source.x = i%elevation_full_image.cols;
                    source.y = i/elevation_full_image.cols;
                    break;
                }
            }
            source_pub_.publish(source);
            geometry_msgs::Point dest;   // last point
            for (int i=0; i < traversability_full_image.rows*traversability_full_image.cols;i++)
            {
                if(traversability_full_image.at<uchar>(i/traversability_full_image.cols,i%traversability_full_image.cols) > 100)
                {
                    dest.x = i%elevation_full_image.cols;
                    dest.y = i/elevation_full_image.cols;
                }
            }
            dest_pub_.publish(dest);

            /// publishig info for metric transformation
            std_msgs::Float32MultiArray msgTransform;
            msgTransform.data.push_back(minX);
            msgTransform.data.push_back(1.0 / resolution_); // here we need  m/pixel
            msgTransform.data.push_back(minY);
            msgTransform.data.push_back(1.0 / resolution_);
            double minZ = -0.6;
            msgTransform.data.push_back(minZ);
            msgTransform.data.push_back(1.0 / zscale);
            transform_pub_.publish(msgTransform);

            /// publishing image_traversability
            sensor_msgs::ImagePtr msgPublish;
            msgPublish = cv_bridge::CvImage(std_msgs::Header(), "mono8", traversability_full_image).toImageMsg();
            img_pub_.publish (msgPublish);

            pub = false;
        }

    }
    else if (verbose_) ROS_INFO_STREAM("not enougth good points : "<<cloud->points.size()<<" points        ");

    if (verbose_) ROS_INFO("Treatment of the PointCloud ok!       ");
    time2 = ros::Time::now();
    duration = time2.toSec() - time0.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec\n              ");
    if (verbose_) cv::waitKey(10);
}

//-------------------------- Used Functions --------------------------------------------------------------------//
// Stolen from voxblox_node that stole it from octomap_manager
bool TraversabilityGraphStitcher::lookupTransformTf(const std::string& from_frame,
                                                    const std::string& to_frame,
                                                    const ros::Time& timestamp,
                                                    voxblox::Transformation* transform) {
    tf::StampedTransform tf_transform;
    ros::Time time_to_lookup = timestamp;
    std::string from_frame_modified = from_frame;

    // If this transform isn't possible at the time, then try to just look up
    // the latest (this is to work with bag files and static transform
    // publisher, etc).
    if (!tf_listener_.canTransform(to_frame, from_frame_modified,
                                   time_to_lookup)) {
        time_to_lookup = ros::Time(0);
        if(verbose_)ROS_WARN("Using latest TF transform instead of timestamp match.");
    }

    try {
        tf_listener_.lookupTransform(to_frame, from_frame_modified, time_to_lookup,
                                     tf_transform);
    } catch (tf::TransformException& ex) {  // NOLINT
        ROS_ERROR_STREAM(
                    "Error getting TF transform from sensor data: " << ex.what());
        return false;
    }

    tf::transformTFToKindr(tf_transform, transform);
    return true;
}

void gammaCorrection(cv::Mat& src, cv::Mat& dst, float fGamma)
{
    unsigned char lut[256];
    for (int i = 0; i < 256; i++)
    {
        lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
    }
    dst = src.clone();
    cv::MatIterator_<uchar> it, end;
    for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
        *it = lut[(*it)];

    return;
}

