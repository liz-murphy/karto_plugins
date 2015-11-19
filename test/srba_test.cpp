/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

//#define SRBA_DETAILED_TIME_PROFILING   1

#include <srba/srba.h>
#include <mrpt/random.h>
#include <mrpt/gui.h>  // For rendering results as a 3D scene

#include <mrpt/graphslam.h> // For global map recovery only
#include <mrpt/opengl/graph_tools.h> // To render the global map

#include <pluginlib/class_loader.h>
#include <slam_karto/localized_range_scan_stamped.h>
#include <ros/ros.h>
#include <slam_karto/slam_solver.h>
#include <open_karto/Mapper.h>


using namespace srba;
using namespace std;
using mrpt::utils::DEG2RAD;
using mrpt::utils::square;

// --------------------------------------------------------------------------------
// A test dataset (generated with https://github.com/jlblancoc/recursive-world-toolkit )
// --------------------------------------------------------------------------------
const double STD_NOISE_XYZ = 0.05;
const double STD_NOISE_ANGLES = mrpt::utils::DEG2RAD(1.0);

struct basic_graph_slam_dataset_entry_t
{
  unsigned int current_kf;
  unsigned int observed_kf;
  double x,y,z, yaw,pitch,roll,  qr,qx,qy,qz; // Relative pose of "observed_kf" as seen from "current_kf"
};
basic_graph_slam_dataset_entry_t dataset[] = {
 {     1,      0,     -1.78055512,      1.11331694,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     2,      1,     -1.71545942,      2.05914961,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
 {     2,      0,     -2.96619160,      3.74601658,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
 {     3,      2,     -1.15014065,      2.45631509,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     4,      3,     -0.71839088,      2.17858845,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     4,      2,     -0.89163376,      4.88530127,      0.00000000,     -0.74799839,     -0.00000000,      0.00000000,      0.93087372,      0.00000000,     -0.00000000,     -0.36534109},
 {     5,      4,     -1.33870852,      1.73631597,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
 {     5,      3,     -1.21151269,      4.02676447,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
 {     6,      5,     -1.67977719,      2.03565806,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     6,      4,     -2.29159821,      4.14103420,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
 {     7,      6,     -1.49006905,      2.30876608,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     8,      7,     -1.15992524,      2.21845386,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
 {     9,      8,     -1.28889269,      1.78614744,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {     9,      7,     -1.55814427,      4.27501619,      0.00000000,     -0.74799802,     -0.00000000,      0.00000000,      0.93087379,      0.00000000,     -0.00000000,     -0.36534092},
 {    10,      9,     -1.67026750,      1.96210498,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {    10,      8,     -2.21751078,      4.09566815,      0.00000000,     -0.74799839,     -0.00000000,      0.00000000,      0.93087372,      0.00000000,     -0.00000000,     -0.36534109},
 {    11,     10,     -1.55210516,      2.27651848,      0.00000000,     -0.37399882,     -0.00000000,      0.00000000,      0.98256650,      0.00000000,     -0.00000000,     -0.18591146},
 {    12,     11,     -1.21625554,      2.27164636,      0.00000000,     -0.37399920,     -0.00000000,      0.00000000,      0.98256647,      0.00000000,     -0.00000000,     -0.18591164},
 {    13,     12,     -1.45455725,      1.51179033,      0.00000000,     -0.22440012,     -0.00000000,      0.00000000,      0.99371217,      0.00000000,     -0.00000000,     -0.11196480},
 {    13,     11,     -2.13482825,      3.99712454,      0.00000000,     -0.59839931,     -0.00000000,      0.00000000,      0.95557270,      0.00000000,     -0.00000000,     -0.29475552},
 {    14,     13,     -2.36655195,      0.41536284,      0.00000000,      0.00000000,     -0.00000000,      0.00000000,      1.00000000,      0.00000000,      0.00000000,      0.00000000},
 {    14,     12,     -3.82110920,      1.92715317,      0.00000000,     -0.22440012,     -0.00000000,      0.00000000,      0.99371217,      0.00000000,     -0.00000000,     -0.11196480},
 {    15,     14,     -2.74448431,     -0.11373775,      0.00000000,      0.00000000,     -0.00000000,      0.00000000,      1.00000000,      0.00000000,      0.00000000,      0.00000000},
 {    15,      0,      4.16910212,      0.67638546,      0.00000000,      1.57079633,     -0.00000000,      0.00000000,      0.70710678,      0.00000000,      0.00000000,      0.70710678},
 {    16,      0,      1.58658626,      0.30349575,      0.00000000,      1.57079633,     -0.00000000,      0.00000000,      0.70710678,      0.00000000,      0.00000000,      0.70710678},
 {    16,     15,     -2.58251586,     -0.37288971,      0.00000000,      0.00000000,     -0.00000000,      0.00000000,      1.00000000,      0.00000000,      0.00000000,      0.00000000},
 {    16,      1,      1.97243380,      2.36770815,      0.00000000,      1.94479552,     -0.00000000,      0.00000000,      0.56332003,      0.00000000,      0.00000000,      0.82623879},
};

class SRBATest
{
public:
  SRBATest();
  ~SRBATest();

  void processDataSet();
  void publishGraphVisualization();
  void visLoop(double vis_publish_period);
private:
  // ROS handles
  ros::NodeHandle node_;
  ros::Publisher marker_publisher_;

  pluginlib::ClassLoader<karto::SLAMSolver> solver_loader_;
  boost::shared_ptr<karto::SLAMSolver> solver_;
  boost::thread* vis_thread_;
};

SRBATest::SRBATest() : solver_loader_("slam_karto", "karto::SLAMSolver") 
{
  try
  {
    solver_ = solver_loader_.createInstance("karto_plugins::SRBASolver");
    ROS_INFO("Loaded SRBA solver plugin");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);

  vis_thread_ = new boost::thread(boost::bind(&SRBATest::visLoop, this, 1.0));
}

SRBATest::~SRBATest()
{
  if (solver_)
    delete solver_.get();

  if (vis_thread_)
  {
    ROS_INFO("Deleting visualization");
    vis_thread_->join();
    delete vis_thread_;
  }
}

void SRBATest::publishGraphVisualization()
{
 visualization_msgs::MarkerArray marray;
 solver_->publishGraphVisualization(marray);
 marker_publisher_.publish(marray);
}

void SRBATest::visLoop(double vis_publish_period)
{
  if(vis_publish_period == 0)
    return;

  ros::Rate r(1.0 / vis_publish_period);
  while(ros::ok())
  {
    publishGraphVisualization();
    r.sleep();
  }
}

void SRBATest::processDataSet()
{
  // Information matrix for relative pose observations:
  {
    Eigen::Matrix<double,6,6> ObsL;
    ObsL.setZero();
    // X,Y,Z:
    for (int i=0;i<3;i++) ObsL(i,i) = 1/square(STD_NOISE_XYZ);
    // Yaw,pitch,roll:
    for (int i=0;i<3;i++) ObsL(3+i,3+i) = 1/square(STD_NOISE_ANGLES);

  }

  // --------------------------------------------------------------------------------
  // Process the dataset:
  // --------------------------------------------------------------------------------
  const size_t nObs = sizeof(dataset)/sizeof(dataset[0]);
  size_t cur_kf = 0; // Start at keyframe #0 in the dataset

  // Add the first node
  karto::RangeReadingsVector empty;
  karto::Name laser_name("laser");
  karto::LocalizedRangeScan scan(laser_name, empty);
  scan.SetUniqueId(dataset[0].current_kf); 
  karto::Vertex<karto::LocalizedRangeScan> vertex(&scan);

  std::cout << "Adding node: " << cur_kf << "\n";
  solver_->AddNode(&vertex);
  std::cout << "Node added\n";
 
  cur_kf = 1; 
  for (size_t obsIdx = 0; obsIdx<nObs;  cur_kf++ /* move to next KF */  )
  {
    std::cout << "ObsIdx: " << obsIdx << " cur_kf: " << cur_kf << "\n";
    // To emulate graph-SLAM, each keyframe MUST have exactly ONE fixed "fake landmark", representing its pose:
    // ------------------------------------------------------------------------------------------------------------
    {
      karto::RangeReadingsVector empty;
      karto::Name laser_name("laser");
      karto::LocalizedRangeScan scan(laser_name, empty);
      scan.SetUniqueId(dataset[obsIdx].current_kf); 
      karto::Vertex<karto::LocalizedRangeScan> vertex(&scan);

      std::cout << "Adding node: " << dataset[obsIdx].current_kf << "\n";
      solver_->AddNode(&vertex);
      std::cout << "Node added\n";
    }
    
    // The rest "observations" are real observations of relative poses:
    // -----------------------------------------------------------------
    while ( dataset[obsIdx].current_kf == cur_kf && obsIdx<nObs )
    {
      std::cout << "Adding observations for " << dataset[obsIdx].current_kf << "\n";
      karto::RangeReadingsVector empty;
      karto::Name laser_name("laser");
      karto::LocalizedRangeScan scan(laser_name, empty);
      scan.SetUniqueId(dataset[obsIdx].current_kf); 
      karto::Vertex<karto::LocalizedRangeScan> vSource(&scan);

      karto::LocalizedRangeScan scan2(laser_name, empty);
      scan2.SetUniqueId(dataset[obsIdx].observed_kf); 
      karto::Vertex<karto::LocalizedRangeScan> vTarget(&scan2);

      std::cout << "Adding edge: " << vSource.GetObject()->GetUniqueId() << " --> " << vTarget.GetObject()->GetUniqueId() << "\n";
      karto::Edge<karto::LocalizedRangeScan> edge(&vSource, &vTarget);

      karto::Pose2 p1(0,0,0);
      karto::Pose2 p2(dataset[obsIdx].x, dataset[obsIdx].y, dataset[obsIdx].yaw);
      karto::Matrix3 covariance; 
      std::cout << "Updating link info\n";
      karto::LinkInfo* plink_info = new karto::LinkInfo(p1,p2,covariance);
      edge.SetLabel(plink_info);
      std::cout << "Adding edge\n";
      solver_->AddConstraint(&edge);
      std::cout << "Edge added\n";
      obsIdx++; // Next dataset entry
    }
  }
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "test_srba");

  SRBATest test;
  test.processDataSet();

  ros::spin();

  return 0; // All ok
}
