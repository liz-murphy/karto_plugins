#include <karto_plugins/srba_solver.h>

namespace karto_plugins {
  SRBASolver::SRBASolver()
  {
    rba_.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose
    rba_.parameters.srba.use_robust_kernel = false;

    // =========== Topology parameters ===========
    rba_.parameters.srba.max_tree_depth       = 3;
    rba_.parameters.srba.max_optimize_depth   = 3;
    rba_.parameters.ecp.submap_size          = 5;
    rba_.parameters.ecp.min_obs_to_loop_closure = 1;

  }

  void const karto::ScanSolver::IdPoseVector& SRBASolver::GetCorrections() const
  {
      return corrections;
  }

  void SRBASolver::Compute()
  {
    corrections.clear();

    typedef std::vector<sba::Node2d, Eigen::aligned_allocator<sba::Node2d> > NodeVector;
    ROS_INFO("Calling SRBA compute");

    // Do nothing here?
  }

  void SRBASolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
  {
    srba_t::new_kf_observation_t obs_field;
    obs_field.is_fixed = true;
    obs_field.obs.feat_id = pVertex->GetObject()->GetUniqueId(); // Feature ID == keyframe ID
    obs_field.obs.obs_data.x = 0;   // Landmark values are actually ignored.
    obs_field.obs.obs_data.y = 0;
    obs_field.obs.obs_data.yaw = 0;
    list_obs.push_back( obs_field );

    srba_t::TNewKeyFrameInfo new_kf_info;
    rba_.define_new_keyframe(
      list_obs,      // Input observations for the new KF
      new_kf_info,   // Output info
      true           // Also run local optimization?
      );

  }
  
  void SRBASolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
  {

    // Need to call create_kf2kf_edge here
    srba_t::new_kf_observation_t obs_field;
    obs_field.is_fixed = false;   // "Landmarks" (relative poses) have unknown relative positions (i.e. treat them as unknowns to be estimated)
    obs_field.is_unknown_with_init_val = false; // Ignored, since all observed "fake landmarks" already have an initialized value.

    obs_field.obs.feat_id      = dataset[obsIdx].observed_kf;
    obs_field.obs.obs_data.x   = dataset[obsIdx].x + mrpt::random::randomGenerator.drawGaussian1D(0,STD_NOISE_XY);
    obs_field.obs.obs_data.y   = dataset[obsIdx].y + mrpt::random::randomGenerator.drawGaussian1D(0,STD_NOISE_XY);
    obs_field.obs.obs_data.yaw = dataset[obsIdx].yaw  + mrpt::random::randomGenerator.drawGaussian1D(0,STD_NOISE_YAW);

    list_obs.push_back( obs_field );
    obsIdx++; // Next dataset entry

    rba_.create_kf2kf_edge(
  }

}   //namespace karto_plugins
