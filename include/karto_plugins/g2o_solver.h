#ifndef KARTO_G2OSOLVER_H
#define KARTO_G2OSOLVER_H

#include <slam_karto/slam_solver.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>
#include <g2o/core/optimizable_graph.h>

namespace g2o
{
  class EdgeSE2;
  class VertexSE2;
  class SparseOptimizer;
  class RawLaser;
  class RobotLaser;
  class LaserParameters;
}

typedef std::pair<int,int> edge_pair_t;
typedef std::map<edge_pair_t,bool> loop_status_t;
typedef std::map<edge_pair_t,visualization_msgs::InteractiveMarker> loop_marker_t;
typedef std::map<edge_pair_t,g2o::OptimizableGraph::Edge *> loop_edge_map_t;

namespace karto_plugins {

  class G2OSolver : public karto::SLAMSolver
  {
  public:
    G2OSolver();
    virtual ~G2OSolver();

  public:
    virtual void Clear();
    virtual void Compute();
    virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

    virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
    virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);
    virtual void getGraph(std::vector<float> &g);

    void publishGraphVisualization(visualization_msgs::MarkerArray &marray);

  protected:
    karto::ScanSolver::IdPoseVector corrections_;
    g2o::SparseOptimizer* optimizer_;
    std::vector<g2o::VertexSE2*> vertices_;

    // parameters
    bool calibration_mode_;
    bool use_interactive_switches_;
    bool online_optimization_;
    int optimization_iterations_;

    // interactive markers support 
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    loop_status_t loop_closure_status_map_;  // keeps track of whether markers are on or off
    loop_marker_t loop_closure_markers_; // allows access to interactive markers by vertex ids of edge
    std::map<edge_pair_t,g2o::OptimizableGraph::Edge *> loop_closure_edges_;
    visualization_msgs::Marker MakeSwitch(visualization_msgs::InteractiveMarker &msg, geometry_msgs::Point &p1, geometry_msgs::Point &p2, bool status);
    visualization_msgs::Marker MakeEdge(visualization_msgs::InteractiveMarker &msg, geometry_msgs::Point &p1, geometry_msgs::Point &p2, bool status);
    virtual bool getEdgeStatus(g2o::OptimizableGraph::Edge* edge);
    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    virtual bool turnEdgeOn(g2o::OptimizableGraph::Edge* e);
    virtual bool turnEdgeOff(g2o::OptimizableGraph::Edge* e);
  };
};

#endif // KARTO_G2OSOLVER_H

