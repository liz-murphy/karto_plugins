#ifndef KARTO_G2OSOLVER_H
#define KARTO_G2OSOLVER_H

#include <slam_karto/slam_solver.h>

typedef std::pair<int,int> edge_pair_t;
typedef std::map<edge_pair_t,bool> loop_status_t;
typedef std::map<edge_pair_t,visualization_msgs::InteractiveMarker> loop_marker_t;
typedef std::map<edge_pair_t,g2o::OptimizableGraph::Edge *> loop_edge_map_t;

namespace karto_plugins {

  class SRBASolver : public karto::SLAMSolver
  {
  public:
    SRBASolver();
    virtual ~SRBASolver();

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

  };
};

#endif // KARTO_G2OSOLVER_H

