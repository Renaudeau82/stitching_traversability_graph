/*-------------------------------------------------------------
 *
 *  Ce code créer un noeud qui permet de calcul de trajectoire dans une carte d'occupation
 *  l'algorithme utilisé est un A* edité par boost
 *
 *
 * ----------------------------------------------------------------*/
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32MultiArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <iostream>


///------------------------------------- objet maze ---------------------------------///
boost::mt19937 random_generator;

// Distance traveled in the maze
typedef double m_distance;

#define GRID_RANK 2
typedef boost::grid_graph<GRID_RANK> grid;
typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

// A hash function for vertices.
struct vertex_hash:std::unary_function<vertex_descriptor, std::size_t> {
    std::size_t operator()(vertex_descriptor const& u) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, u[0]);
        boost::hash_combine(seed, u[1]);
        return seed;
    }
};

typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type
filtered_grid;

// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
//
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.
class maze {
public:
    friend std::ostream& operator<<(std::ostream&, const maze&);
    friend maze random_maze(std::size_t, std::size_t);
    // constructor
    maze():m_grid(create_grid(0, 0)),m_barrier_grid(create_barrier_grid()) {}
    maze(std::size_t x, std::size_t y):m_grid(create_grid(x, y)),m_barrier_grid(create_barrier_grid()) {}
    maze(cv::Mat In_image);

    vertex_descriptor source;
    vertex_descriptor goal;
    std::deque<vertex_descriptor> soluce_vect;

    int _x,_y;

    // Try to find a path from the lower-left-hand corner source (0,0) to the
    // upper-right-hand corner goal (x-1, y-1).
    bool solve();
    bool solved() const {return !m_solution.empty();}
    bool solution_contains(vertex_descriptor u) const {return m_solution.find(u) != m_solution.end();}
    const std::deque<vertex_descriptor> get_solution(){return soluce_vect;}
    cv::Mat to_image();
    // The length of the maze along the specified dimension.
    vertices_size_type length(std::size_t d) const {return m_grid.length(d);}
    bool has_barrier(vertex_descriptor u) const {return m_barriers.find(u) != m_barriers.end();}

private:
    // Create the underlying rank-2 grid with the specified dimensions.
    grid create_grid(std::size_t x, std::size_t y)
    {
        boost::array<std::size_t, GRID_RANK> lengths = { {x, y} };
        return grid(lengths);
    }
    // Filter the barrier vertices out of the underlying grid.
    filtered_grid create_barrier_grid()
    {
        return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
    }
    // The grid underlying the maze
    grid m_grid;
    // The underlying maze grid with barrier vertices filtered out
    filtered_grid m_barrier_grid;
    // The barriers in the maze
    vertex_set m_barriers;
    // The vertices on a solution path through the maze
    vertex_set m_solution;
    // The length of the solution path
    m_distance m_solution_length;
};


// Euclidean heuristic for a grid
//
// This calculates the Euclidean m_distance between a vertex and a goal vertex.
class euclidean_heuristic:public boost::astar_heuristic<filtered_grid, double>
{
public:
    euclidean_heuristic(vertex_descriptor goal):m_goal(goal) {}

    double operator()(vertex_descriptor v)
    {
        int g0 = int(m_goal[0]);
        int g1 = int(m_goal[1]);
        int v0 = int(v[0]);
        int v1 = int(v[1]);
        int d0 = g0-v0;
        int d1 = g1-v1;
        return sqrt(d0*d0+d1*d1);
    }

private:
    vertex_descriptor m_goal;
};

// Exception thrown when the goal vertex is found
struct found_goal {};

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor:public boost::default_astar_visitor
{
    astar_goal_visitor(vertex_descriptor goal):m_goal(goal) {}

    void examine_vertex(vertex_descriptor u, const filtered_grid&)
    {
        if (u == m_goal)
            throw found_goal();
    }

private:
    vertex_descriptor m_goal;
};

// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
    // constant weight
    boost::static_property_map<m_distance> weight(1.0);
    // The predecessor map is a vertex-to-vertex mapping.
    typedef boost::unordered_map<vertex_descriptor,
            vertex_descriptor,
            vertex_hash> pred_map;
    pred_map predecessor;
    boost::associative_property_map<pred_map> pred_pmap(predecessor);
    // The m_distance map is a vertex-to-m_distance mapping.
    typedef boost::unordered_map<vertex_descriptor,
            m_distance,
            vertex_hash> dist_map;
    dist_map distance;
    boost::associative_property_map<dist_map> dist_pmap(distance);

    vertex_descriptor s = source;
    vertex_descriptor g = goal;
    euclidean_heuristic heuristic(g);
    astar_goal_visitor visitor(g);

    // try to find a solutino
    try {
        astar_search(m_barrier_grid, s, heuristic,
                     boost::weight_map(weight).
                     predecessor_map(pred_pmap).
                     distance_map(dist_pmap).
                     visitor(visitor) );
    }
    catch(found_goal fg) // if the goal is reached
    {
        // Walk backwards from the goal through the predecessor chain adding
        // vertices to the solution path.
        for (vertex_descriptor u = g; u != s; u = predecessor[u])
        {
            m_solution.insert(u);
            soluce_vect.push_front(u);
        }
        m_solution.insert(s);
        soluce_vect.push_front(s);
        m_solution_length = distance[g];
        return true;
    }

    return false;
}


#define BARRIER "#"
// Print the maze as an ASCII map.
std::ostream& operator<<(std::ostream& output, const maze& m)
{
    // Header
    for (vertices_size_type i = 0; i < m.length(0)+2; i++)
        output << BARRIER;
    output << std::endl;
    // Body
    for (int y = m.length(1)-1; y >= 0; y--) {
        // Enumerate rows in reverse order and columns in regular order so that
        // (0,0) appears in the lower left-hand corner.  This requires that y be
        // int and not the unsigned vertices_size_type because the loop exit
        // condition is y==-1.
        for (vertices_size_type x = 0; x < m.length(0); x++) {
            // Put a barrier on the left-hand side.
            if (x == 0)
                output << BARRIER;
            // Put the character representing this point in the maze grid.
            vertex_descriptor u = {{x, vertices_size_type(y)}};
            if (u==m.source)
                output << "s";
            else if (u==m.goal)
                output << "g";
            else if (m.solution_contains(u))
                output << ".";
            else if (m.has_barrier(u))
                output << BARRIER;
            else
                output << " ";
            // Put a barrier on the right-hand side.
            if (x == m.length(0)-1)
                output << BARRIER;
        }
        // Put a newline after every row except the last one.
        output << std::endl;
    }
    // Footer
    for (vertices_size_type i = 0; i < m.length(0)+2; i++)
        output << BARRIER;
    output << std::endl;
    if (m.solved())
        output << std::endl << "Solution length " << m.m_solution_length;
    output << std::endl;
    return output;
}

// Generate a maze from cv::Mat
maze::maze(cv::Mat In_image):m_grid(create_grid(In_image.cols, In_image.rows)), m_barrier_grid(create_barrier_grid()) {

    _x=In_image.rows;
    _y=In_image.cols;
    for(vertices_size_type i=0; i<In_image.cols; i++)
    {
        for(vertices_size_type j=0; j<In_image.rows; j++)
        {
            int a=  In_image.at<unsigned char>(j,i);
            if(a<128)
            {
                vertex_descriptor u = {{i, j}};
                if (!has_barrier(u))
                    m_barriers.insert(u);
            }
        }
    }

}

// Generate image of the maze and the result
cv::Mat maze::to_image()
{
    cv::Mat Out(_x,_y,CV_8UC3,cv::Scalar(255,255,255));
    for (vertices_size_type y = 0; y < _x; y++) {

        for (vertices_size_type x = 0; x < _y; x++) {
            // Put the character representing this point in the maze grid.
            vertex_descriptor u = {{x, y}};
            if (u==source)
                cv::circle(Out,cv::Point(x,y),3,cv::Scalar(0,255,0),-1); //Out.at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,0);
            else if (u==goal)
                cv::circle(Out,cv::Point(x,y),3,cv::Scalar(0,0,255),-1); //Out.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);
            else if (solution_contains(u))
                Out.at<cv::Vec3b>(y,x) = cv::Vec3b(255,0,0);
            else if (has_barrier(u))
                Out.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
        }

    }

    return Out.clone();
}

///------------------------------------ Class worker ---------------------------------------------//
class Worker
{
private:
    ros::NodeHandle nh; ///< Node Handle
    image_transport::ImageTransport it; ///< Image transport handle

    image_transport::Subscriber pCS; ///< Subscriber occupation images
    ros::Subscriber sub_source_; ///< Subscriber to stating point
    ros::Subscriber sub_dest_; ///< Subscriber to goal point

    image_transport::Publisher pImage; ///< Publisher of result image
    ros::Publisher pTraj; ///< Publisher trajectory

    // global variables
    cv::Point pDest_;
    cv::Point pSource_;
    bool new_traj;
    // param
    int step;

public:
    Worker(); ///< Constructeur du worker
    void cbNewCSimg(const sensor_msgs::ImageConstPtr &msg); ///< Callback for new occupancy image
    void cbPtSource(geometry_msgs::Point::ConstPtr msg); ///< Callback for  new start point
    void cbPtDest(geometry_msgs::Point::ConstPtr msg); ///< Callback for new goal point
};

Worker::Worker():nh("~"),it(nh)
{
    // subscribers
    sub_dest_ = nh.subscribe("/pt_dest",1,&Worker::cbPtDest,this);
    sub_source_ = nh.subscribe("/pt_source",1,&Worker::cbPtSource,this);
    pCS = it.subscribe("/image_traversability",1,&Worker::cbNewCSimg,this);
    // publishers
    pImage = it.advertise("/a_star_image",1);
    pTraj = nh.advertise<std_msgs::Int32MultiArray>("/trajectory_points",1);
    // variables
    new_traj=false;
    pDest_ = cv::Point(0,0);
    pSource_ = cv::Point(0,0);
    // params
    nh.param("step_size",step,10);
}

void Worker::cbNewCSimg(const sensor_msgs::ImageConstPtr &msg)
{
    if(new_traj)
    {
        ros::Time time0, time1;
        double duration;
        time0 = ros::Time::now();

        cv_bridge::CvImagePtr input_bridge;
        std_msgs::Header H = msg->header;

        try {
            input_bridge = cv_bridge::toCvCopy(msg, "mono8");
        }
        catch (cv_bridge::Exception& ex){
            ROS_ERROR_STREAM(ex.what());
            return;
        }

        /// retreave image from msg
        cv::Mat IN_img=input_bridge->image.clone();
        //cv::imshow("input image",IN_img);
        //cv::waitKey(100);

        /// create lab from image
        maze maze_graph(IN_img);
        // computation time
        time1 = ros::Time::now();
        duration = time1.toSec() - time0.toSec();
        ROS_INFO_STREAM("Received image, create maze in "<<duration<<"sec");

        /// define starting and goal points
        maze_graph.source = {{pSource_.x, pSource_.y}};
        maze_graph.goal = {{pDest_.x, pDest_.y}};

        /// solve
        time0 = ros::Time::now();
        if(IN_img.at<unsigned char>(pDest_.x,pDest_.y)>128)
            maze_graph.solve();
        // computation time
        time1 = ros::Time::now();
        duration = time1.toSec() - time0.toSec();
        ROS_INFO_STREAM("try to solve A* : "<<duration<<"sec");

        /// Create image solution and publish
        cv::Mat OUT_img;
        OUT_img = maze_graph.to_image();
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(H, "bgr8",OUT_img).toImageMsg();
        pImage.publish(img_msg);

        if(!maze_graph.solved())
        {
            ROS_INFO("A star have no solution !");
            return;
        }

        /// create sampled trajectory and publish
        std::deque<vertex_descriptor> traj = maze_graph.get_solution();
        // if solution but not trajectry just return
        if(traj.size()==0)
        {
            ROS_INFO("A star have no trajectory !");
            return;
        }
        std_msgs::Int32MultiArray traj_msg;
        for(int i=0; i< traj.size();i++)
        {
            traj_msg.data.push_back(traj[i][0]);
            traj_msg.data.push_back(traj[i][1]);
        }
        pTraj.publish(traj_msg);

        new_traj=false;
    }
}

void Worker::cbPtSource(geometry_msgs::Point::ConstPtr msg)
{
    cv::Point Pin;
    Pin.x=(int)round(msg->x);
    Pin.y=(int)round(msg->y);
    if(Pin!=pSource_)
    {
        pSource_ = Pin;
        new_traj = true;
    }
}

void Worker::cbPtDest(geometry_msgs::Point::ConstPtr msg)
{
    cv::Point Pin;
    Pin.x=(int)round(msg->x);
    Pin.y=(int)round(msg->y);
    if(Pin!=pDest_)
    {
        pDest_ = Pin;
        new_traj = true;
    }
}

///--------------------------------------- Main function for the node ----------------------------------///
int main(int argc, char ** argv)
{
    // Initialisation of ROS node
    ros::init(argc,argv,"A_star_planner");

    // call worker class
    Worker worker;

    ros::spin();
}
