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
    srba_t::new_kf_observations_t  list_obs;
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

    if(pVertex->GetObject()->GetUniqueId() != new_kf_info.kf_id)
      ROS_ERROR("Key frame id's DO NOT MATCH");

  }
  
  void SRBASolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
  {

    // Need to call create_kf2kf_edge here
    srba_t::new_kf_observations_t  list_obs;
    srba_t::new_kf_observation_t obs_field;
    obs_field.is_fixed = false;   // "Landmarks" (relative poses) have unknown relative positions (i.e. treat them as unknowns to be estimated)
    obs_field.is_unknown_with_init_val = false; // Ignored, since all observed "fake landmarks" already have an initialized value.

    karto::LocalizedRangeScan* pSource = pEdge->GetSource()->GetObject();
    karto::LocalizedRangeScan* pTarget = pEdge->GetTarget()->GetObject();
    karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

    karto::Pose2 diff = pLinkInfo->GetPoseDifference();
    g2o::SE2 motion(diff.GetX(), diff.GetY(), diff.GetHeading());

    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
    Eigen::Matrix<double,3,3> m;
    m(0,0) = precisionMatrix(0,0);
    m(0,1) = m(1,0) = precisionMatrix(0,1);
    m(0,2) = m(2,0) = precisionMatrix(0,2);
    m(1,1) = precisionMatrix(1,1);
    m(1,2) = m(2,1) = precisionMatrix(1,2);
    m(2,2) = precisionMatrix(2,2);

    edge->vertices()[0] = optimizer_->vertices().find(pSource->GetUniqueId())->second;
    edge->vertices()[1] = optimizer_->vertices().find(pTarget->GetUniqueId())->second;

    obs_field.obs.feat_id      = pEdge->GetSource()->GetObject();  // Is this right??
    obs_field.obs.obs_data.x   = diff.GetX();
    obs_field.obs.obs_data.y   = diff.GetY();
    obs_field.obs.obs_data.yaw = diff.GetZ();

    list_obs.push_back( obs_field );

    //std::vector<TNewEdgeInfo> new_k2k_edge_id;
    //rba_.determine_kf2kf_edges_to_create(new_kf_id,obs, new_k2k_edge_ids);
    //rba_.add_observation( new_kf_id, it_obs->obs, fixed_rel_pos, unk_rel_pos_initval );

    TPairKeyFrameID new_edge(pSource->GetUniqueId()->second, pTarget->GetUniqueId()->second);

    rba_.create_kf2kf_edge(pTarget->GetUniqueId()->second,
        new_edge,
        list_obs);


  }

}   //namespace karto_plugins
