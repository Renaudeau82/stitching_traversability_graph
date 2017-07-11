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
    if (verbose_) ROS_INFO_STREAM("PointCloud received, starting treatment.");
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
            point_C(0,0) = cloud->points[pt_idx].x; //const pcl::PointXYZ
            point_C(1,0) = cloud->points[pt_idx].y; //const pcl::PointXYZ
            point_C(2,0) = cloud->points[pt_idx].z; //const pcl::PointXYZ
            Eigen::Matrix<double, 3, 1>  point_G = T_G_C * point_C;
            cloud->points[pt_idx].x = point_G(0,0);
            cloud->points[pt_idx].y = point_G(1,0);
            cloud->points[pt_idx].z = point_G(2,0);
        }
    }
    else return;

    /// clean outliers ( 2.5<x<21 && 25<y<45 && -0.6<z<2.0 ) // zone position !!!
    pcl::PointIndices::Ptr indices_in (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> eifilter (false);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZ point = cloud->points[i];
        if(point.data[0]>2.5 && point.data[0]< 21 && point.data[1]>25 && point.data[1]<45 && point.data[2]>-0.6 && point.data[2]< 2)
            indices_in->indices.push_back(i);
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
        if (verbose_) ROS_INFO_STREAM("Compution of pcd_image : "<<cloud->points.size()<<" points");
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
        zscale = 100; // 255/(zmax-zmin); // ça serait bien d'avoir une echelle fixe (paramêtrage zmin = -0.6 connu, paramétrage zmax définit à 1,95m)
        if (verbose_) ROS_INFO_STREAM("zone : "<<xmin<<"-"<<xmax<<" , "<<ymin<<"-"<<ymax<<" , "<<zmin<<"-"<<zmax<<" -> scale="<<resolution_<<" ("<<width<<","<<height<<") zscale="<<zscale);
        if(width < 0 || height < 0) return;
        cv::Mat elevation_tmp = cv::Mat::zeros(height, width, CV_8UC1);
        cv::Mat pcd_in_tmp = cv::Mat::zeros(height, width, CV_8UC1);
        // projection de la profondeur des points dans l'image
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZ point = cloud->points[i];
            unsigned int y = (unsigned int)(resolution_*(point.data[1]-ymin));
            unsigned int x = (unsigned int)(resolution_*(point.data[0]-xmin)); //(width-1)-
            if(y<0 || y>height-1) ROS_ERROR_STREAM("erreur en y : "<<y);
            if(x<0 || x>width-1) ROS_ERROR_STREAM("erreur en x : "<<x);
            if(y<3/4.0*height && y>1/4.0*height) // on se concentre sur le milieu de l'image (on evite ainsi les effet de bords
            {
            elevation_tmp.at<unsigned char>(y,x) = (unsigned char)(zscale*(cloud->points[i].data[2] + 0.6)); // +0.6 = -zmin
            pcd_in_tmp.at<unsigned char>(y,x) = 255;
            }
        }
        // améliorer le contraste depth and fill elevation_map holes
        gammaCorrection(elevation_tmp, elevation_tmp, gamma_); //2.0 c'est trop, 1.0 c'est pas assez
        //cv::GaussianBlur(elevation_tmp,elevation_tmp,cv::Size(5,5),0,0);
        //if (verbose_) cv::imshow("elevation_tmp",elevation_tmp);
        // fill the holes of the in_map
        int element_size0 = 1;
        cv::Mat element0 = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*element_size0 + 1, 2*element_size0+1 ), cv::Point( element_size0, element_size0 ) );
        cv::morphologyEx( pcd_in_tmp, pcd_in_tmp, cv::MORPH_DILATE, element0 ); // on enlève les moyenne taches blanche dans le noir
        cv::medianBlur(pcd_in_tmp,pcd_in_tmp,3);
        time2 = ros::Time::now();
        duration = time2.toSec() - time1.toSec();
        if(verbose_) ROS_INFO_STREAM(duration<<"sec");

        // affichage histogramme
        //cv::Mat hst;
        //Renaudeau::Opencv::histogram(elevation_tmp,hst);
        //cv::imshow("hist tmp", hst);

        /// Initialisation de la carte global avec la première image correcte
        if(first)
        {
            minX = xmin;
            maxX = xmax;
            minY = ymin;
            maxY = ymax;
            elevation_full_image = elevation_tmp;
            pcd_in_full_image = pcd_in_tmp;
            first = false;
        }
        /// stitching de l'image d'elevation tmp sur la carte globale
        else
        {/**/
            if (verbose_) ROS_INFO("Stitching");
            time1 = ros::Time::now();
            // definition de la nouvelle image d'elevation
            int width = resolution_*(std::max(xmax,maxX)-std::min(xmin,minX)) +1;
            int height = resolution_*(std::max(ymax,maxY)-std::min(ymin,minY)) +1;
            cv::Mat elevation_new = cv::Mat::zeros(height, width, CV_8UC1);
            cv::Mat pcd_in_new = cv::Mat::zeros(height, width, CV_8UC1);
            // copie de elevation full
            for(unsigned int i=0;i<elevation_full_image.rows;i++)
            {
                for(unsigned int j=0;j<elevation_full_image.cols;j++)
                {
                    unsigned int I = (unsigned int) i+std::max(0.0,(minY-ymin)*resolution_);
                    unsigned int J = (unsigned int) j+std::max(0.0,(minX-xmin)*resolution_);
                    if(I<0 || I>height-1) ROS_ERROR_STREAM("erreur en i : "<<I);
                    if(J<0 || J> width-1) ROS_ERROR_STREAM("erreur en j : "<<J);
                    elevation_new.at<unsigned char>(I,J) = elevation_full_image.at<unsigned char>(i,j);
                    pcd_in_new.at<unsigned char>(I,J) = pcd_in_full_image.at<unsigned char>(i,j);
                }
            }
            // Update !  copie de elevation tmp
            bool kernel = true;
            double kernel_coef = (2.0/elevation_tmp.cols) * (2.0/elevation_tmp.cols); // alpha = 1 if  dist=width/2
            if(verbose_) ROS_INFO_STREAM("kernel_coef = "<<kernel_coef);
            for(unsigned int i=0;i<elevation_tmp.rows;i++)
            {
                for(unsigned int j=0;j<elevation_tmp.cols;j++)
                {
                    if(elevation_tmp.at<unsigned char>(i,j) > 0)
                    {
                        unsigned int I = (unsigned int) i+std::max(0.0,(ymin-minY)*resolution_);
                        unsigned int J = (unsigned int) j+std::max(0.0,(xmin-minX)*resolution_);
                        //J = width-J;
                        if(I<0 || I>height-1) ROS_ERROR_STREAM("erreur en i : "<<I);
                        if(J<0 || J> width-1) ROS_ERROR_STREAM("erreur en j : "<<J);
                        if(kernel)
                        {
                            double dist = (elevation_tmp.cols/2.0 - j)*(elevation_tmp.cols/2.0 - j) + (elevation_tmp.rows/2.0 - i)*(elevation_tmp.rows/2.0 - i);
                            double alpha = std::min(std::max(dist*kernel_coef,0.0),0.7);
                            elevation_new.at<unsigned char>(I,J) = alpha*elevation_new.at<unsigned char>(I,J) + (1-alpha)*(elevation_tmp.at<unsigned char>(i,j)+40); // + 40 offset pour visualisation
                        }
                        else
                        {
                            elevation_new.at<unsigned char>(I,J) = (elevation_new.at<unsigned char>(I,J) + elevation_tmp.at<unsigned char>(i,j) + 40)/2;
                        }
                        pcd_in_new.at<unsigned char>(I,J) = pcd_in_tmp.at<unsigned char>(i,j);
                    }
                }
            }
            // actualisation
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
            if (verbose_) ROS_INFO_STREAM("zone : "<<minX<<"-"<<maxX<<" , "<<minY<<"-"<<maxY<<" -> scale="<<resolution_<<" ("<<width<<","<<height<<") ");
            /**/
        }

        if (verbose_) cv::imshow("elevation_full",elevation_full_image);
/**/
        /// Utilisation d'Opencv pour traiter l'image de profondeur
        if (verbose_) ROS_INFO("Compution normals");
        time1 = ros::Time::now();
        cv::Mat modifiedImage = elevation_full_image;
        // gradiant
        int ddepth = CV_32F;
        cv::Mat grad_x, grad_y;
        // Gradient X et Y
        cv::Sobel( modifiedImage, grad_x, ddepth, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
        cv::Sobel( modifiedImage, grad_y, ddepth, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
        cv::Mat gradiant = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F);
        cv::magnitude(grad_x,grad_y,gradiant);
        cv::normalize(gradiant, gradiant,0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
        int amplify_magnitude = 10;//20; // pour la visualisation couleur
        for(unsigned int i=0;i<grad_x.rows;i++)
        {
            for(unsigned int j=0;j<grad_y.cols;j++)
            {
                gradiant.at<uchar>(i,j) = gradiant.at<uchar>(i,j)*amplify_magnitude;
            }
        }
        // filtrage gaussien sur le gradiant pour élargir les zones dangereuse et réduir les pics
        cv::GaussianBlur(gradiant,gradiant,cv::Size(3,3),0,0);
        // orientation
        cv::Mat orientation = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F); //to store the gradients grad_x.convertTo(grad_x,CV_32F);
        grad_y.convertTo(grad_y,CV_32F);
        grad_x.convertTo(grad_x,CV_32F);
        cv::phase(grad_x, grad_y, orientation,true);
        cv::normalize(orientation, orientation, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
        // création de nouvelle couches pour la normal_z à l'image modifiée
        cv::Mat normal_z = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F);
        for (int i=0; i < modifiedImage.rows;i++)
        {
            for (int j=0; j < modifiedImage.cols;j++)
            {
                Eigen::Vector3d normal(sin(gradiant.at<uchar>(i,j)/255.0) * sin(orientation.at<uchar>(i,j)*2*M_PI/255.0) , sin(gradiant.at<uchar>(i,j)/255.0)  * cos(orientation.at<uchar>(i,j)*2*M_PI/255.0) , 1-gradiant.at<uchar>(i,j)/255.0);
                normal.normalize();
                normal_z.at<float>(i,j) = (float)normal.z();
            }
        }
        cv::normalize(normal_z, normal_z, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
        time2 = ros::Time::now();
        duration = time2.toSec() - time1.toSec();
        if(verbose_) ROS_INFO_STREAM(duration<<"sec");
        // dessin couleur
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
        if(verbose_) cv::imshow("fusion", fusion);
        //cv::imshow("gradiant",gradiant);
        //cv::imshow("orientation",orientation);
        //cv::imshow("normal_z",normal_z);

        /// Computation of image traversability
        if (verbose_) ROS_INFO("Compution traversability");
        time1 = ros::Time::now();
        traversability_full_image = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_8U);
        // selectiond des point traversable (thresholding)
        for (int i=0; i < modifiedImage.rows;i++)
        {
            for (int j=0; j < modifiedImage.cols;j++)
            {
                if(elevation_full_image.at<uchar>(i,j) < 100 && pcd_in_full_image.at<uchar>(i,j) > 0  && normal_z.at<uchar>(i,j) > 0.85*255)
                    traversability_full_image.at<uchar>(i,modifiedImage.cols-j+1) = 255; // flip image to be z downward
            }
        }
        if (verbose_) cv::imshow("traversability_image_raw",traversability_full_image);
        // median filter to remove small black pixels
        cv::medianBlur(traversability_full_image,traversability_full_image,5);
        // erode to make bigger the places to evode depanding on the robot_size
        int element_size = (int)robot_size_/2;
        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*element_size + 1, 2*element_size+1 ), cv::Point( element_size, element_size ) );
        cv::morphologyEx( traversability_full_image, traversability_full_image, cv::MORPH_ERODE, element ); // on enlève les moyenne taches blanche dans le noir
        time2 = ros::Time::now();
        duration = time2.toSec() - time1.toSec();
        if (verbose_) ROS_INFO_STREAM(duration<<"sec");
        //cv::imshow("traversability",traversability_full_image);
        //cv::imwrite("/home/eth/traversability_opencv.png", traversability_full_image);

        /**/
    }
    else if (verbose_) ROS_INFO_STREAM("not enougth good points : "<<cloud->points.size()<<" points");

    //--------------------- Publishing ---------------------------------------------------//
    if(automatic_pub_ || pub)
    {
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
        //dest.x = traversability_full_image.cols/2;
        //dest.y = traversability_full_image.rows/2;
        dest_pub_.publish(dest);

        /// publishig info for metric transformation
        std_msgs::Float32MultiArray msgTransform;
        msgTransform.data.push_back(minX);
        msgTransform.data.push_back(1.0 / resolution_); // on veut m/pixel
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

    if (verbose_) ROS_INFO("Treatment of the PointCloud ok!");
    time2 = ros::Time::now();
    duration = time2.toSec() - time0.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec\n");
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

