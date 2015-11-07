#include <karto_plugins/srba_solver.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <mrpt/gui.h>  // For rendering results as a 3D scene

PLUGINLIB_EXPORT_CLASS(karto_plugins::SRBASolver, karto::SLAMSolver)

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

    first_edge_= true;
  }

  SRBASolver::~SRBASolver()
  {
  }

  const karto::ScanSolver::IdPoseVector& SRBASolver::GetCorrections() const
  {
      return corrections_;
  }

  void SRBASolver::Compute()
  {
    //corrections.clear();

    //typedef std::vector<sba::Node2d, Eigen::aligned_allocator<sba::Node2d> > NodeVector;
    //ROS_INFO("Calling SRBA compute");

    // Do nothing here?
  }

  void SRBASolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
  { 
    if(first_edge_)
      return;

    //srba_t::new_kf_observations_t  list_obs;
    srba_t::new_kf_observation_t obs_field;
    obs_field.is_fixed = true;
    obs_field.obs.feat_id = pVertex->GetObject()->GetUniqueId(); // Feature ID == keyframe ID
    obs_field.obs.obs_data.x = 0;   // Landmark values are actually ignored.
    obs_field.obs.obs_data.y = 0;
    obs_field.obs.obs_data.yaw = 0;
    list_obs_.push_back( obs_field );

    srba_t::TNewKeyFrameInfo new_kf_info;
    ROS_INFO("Adding node with id: %d", (int)pVertex->GetObject()->GetUniqueId());
    rba_.define_new_keyframe(
      list_obs_,      // Input observations for the new KF
      new_kf_info,   // Output info
      true           // Also run local optimization?
      );

    last_kf_id_ = new_kf_info.kf_id;
    list_obs_.clear();

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

    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
    Eigen::Matrix<double,3,3> m;
    m(0,0) = precisionMatrix(0,0);
    m(0,1) = m(1,0) = precisionMatrix(0,1);
    m(0,2) = m(2,0) = precisionMatrix(0,2);
    m(1,1) = precisionMatrix(1,1);
    m(1,2) = m(2,1) = precisionMatrix(1,2);
    m(2,2) = precisionMatrix(2,2);

    obs_field.obs.feat_id      = pSource->GetUniqueId();  // Is this right??
    obs_field.obs.obs_data.x   = diff.GetX();
    obs_field.obs.obs_data.y   = diff.GetY();
    obs_field.obs.obs_data.yaw = diff.GetHeading();

    list_obs_.push_back( obs_field );

    //std::vector<TNewEdgeInfo> new_k2k_edge_id;
    //rba_.determine_kf2kf_edges_to_create(new_kf_id,obs, new_k2k_edge_ids);
    //rba_.add_observation( new_kf_id, it_obs->obs, fixed_rel_pos, unk_rel_pos_initval );

    ROS_INFO("Adding edge between %d and %d", pSource->GetUniqueId(), pTarget->GetUniqueId());
    TPairKeyFrameID new_edge(pSource->GetUniqueId(), pTarget->GetUniqueId());

/*    rba_.create_kf2kf_edge(pTarget->GetUniqueId(),
        new_edge,
        list_obs);*/
    if(first_edge_)
      first_edge_ = false;
  }

  void SRBASolver::publishGraphVisualization(visualization_msgs::MarkerArray &marray)
  {
    mrpt::gui::CDisplayWindow3D win("RBA results",640,480);

    // Do nothing
    //     // --------------------------------------------------------------------------------
    // Show 3D view of the resulting map:
    // --------------------------------------------------------------------------------
    srba_t::TOpenGLRepresentationOptions  opengl_options;
    mrpt::opengl::CSetOfObjectsPtr rba_3d = mrpt::opengl::CSetOfObjects::Create();

    rba_.build_opengl_representation(
      last_kf_id_,  // Root KF: the current (latest) KF
      opengl_options, // Rendering options
      rba_3d  // Output scene
      );

    {
      mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();
      scene->clear();
      scene->insert(rba_3d);
      win.unlockAccess3DScene();
    }
    win.repaint();

    //std::cout << "Press any key to continue.\n";
    //win.waitForKey();

  }

  void SRBASolver::getGraph(std::vector<float> &g)
  {
  }

  void SRBASolver::Clear()
  {
  }

}   //namespace karto_plugins
