#ifndef KARTO_G2OSOLVER_H
#define KARTO_G2OSOLVER_H

#include <slam_karto/slam_solver.h>

struct RBA_OPTIONS : public RBA_OPTIONS_DEFAULT
{
  //  typedef ecps::local_areas_fixed_size            edge_creation_policy_t;  //!< One of the most important choices: how to construct the relative coordinates graph problem
  //  typedef options::sensor_pose_on_robot_none      sensor_pose_on_robot_t;  //!< The sensor pose coincides with the robot pose
  typedef options::observation_noise_constant_matrix<observations::RelativePoses_2D>   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to some given matrix
  //  typedef options::solver_LM_schur_dense_cholesky solver_t;                //!< Solver algorithm (Default: Lev-Marq, with Schur, with dense Cholesky)
};

typedef RbaEngine<
  kf2kf_poses::SE2,               // Parameterization  of KF-to-KF poses
  landmarks::RelativePoses2D,     // Parameterization of landmark positions
  observations::RelativePoses_2D, // Type of observations
  RBA_OPTIONS
  >  srba_t;

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
    srba_t rba_;

  };
};

#endif // KARTO_G2OSOLVER_H

