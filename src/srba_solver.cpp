#include <karto_plugins/srba_solver.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <mrpt/gui.h>  // For rendering results as a 3D scene
#include <string>
PLUGINLIB_EXPORT_CLASS(karto_plugins::SRBASolver, karto::SLAMSolver)

using namespace srba;
using mrpt::poses::CPose3D;

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

    first_keyframe_ = true;
    first_edge_ = true;
    curr_kf_id_ = 0;
  }

  SRBASolver::~SRBASolver()
  {
  }

  const karto::ScanSolver::IdPoseVector& SRBASolver::GetCorrections() const
  {
      ROS_ERROR("Corrections called");
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
    //srba_t::new_kf_observations_t  list_obs;
    srba_t::new_kf_observation_t obs_field;
    obs_field.is_fixed = true;
    ROS_INFO("Adding node with kf_id %d", curr_kf_id_);
    obs_field.obs.feat_id = curr_kf_id_; // Feature ID == keyframe ID
    obs_field.obs.obs_data.x = 0;   // Landmark values are actually ignored.
    obs_field.obs.obs_data.y = 0;
    obs_field.obs.obs_data.yaw = 0;
    list_obs_.push_back( obs_field );

    if(first_keyframe_)
      first_keyframe_ = false;
    else
    {
      // Add the last keyframe
      srba_t::TNewKeyFrameInfo new_kf_info;
      ROS_INFO("Adding node with id: %d", curr_kf_id_);
      rba_.define_new_keyframe(
      list_obs_,      // Input observations for the new KF
      new_kf_info,   // Output info
      true // Also run local optimization?
      );
      ROS_ASSERT(cur_kf_id_ == new_kf_info.kf_id);
    }
    curr_kf_id_ = pVertex->GetObject()->GetUniqueId();
    list_obs_.clear();

  }
  
  void SRBASolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
  {
  
    first_edge_ = false;
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

    obs_field.obs.feat_id      = pTarget->GetUniqueId();  // Is this right??
    obs_field.obs.obs_data.x   = diff.GetX();
    obs_field.obs.obs_data.y   = diff.GetY();
    obs_field.obs.obs_data.yaw = diff.GetHeading();

    list_obs_.push_back( obs_field );

    ROS_INFO("Adding edge between %d and %d", curr_kf_id_, pTarget->GetUniqueId());
    ROS_INFO("%f, %f, %f", diff.GetX(), diff.GetY(), diff.GetHeading());
  }

  void SRBASolver::publishGraphVisualization(visualization_msgs::MarkerArray &marray)
  { 
    // Vertices are round, red spheres
    visualization_msgs::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.ns = "karto";
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.scale.z = 0.15;
    m.color.r = 1.0;
    m.color.g = 0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.lifetime = ros::Duration(0);

    // Odometry edges are opaque blue line strips 
    visualization_msgs::Marker edge;
    edge.header.frame_id = "/map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.ns = "karto";
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;
    edge.color.a = 1.0;
    edge.color.r = 0.0;
    edge.color.g = 0.0;
    edge.color.b = 1.0;
  
    // Loop edges are purple, opacity depends on backend state
    visualization_msgs::Marker loop_edge;
    loop_edge.header.frame_id = "/map";
    loop_edge.header.stamp = ros::Time::now();
    loop_edge.action = visualization_msgs::Marker::ADD;
    loop_edge.ns = "spanning_tree";
    loop_edge.id = 0;
    loop_edge.type = visualization_msgs::Marker::LINE_STRIP;
    loop_edge.scale.x = 0.1;
    loop_edge.scale.y = 0.1;
    loop_edge.scale.z = 0.1;
    loop_edge.color.a = 1.0;
    loop_edge.color.r = 1.0;
    loop_edge.color.g = 0.0;
    loop_edge.color.b = 1.0;
   
    visualization_msgs::Marker node_text;
    node_text.header.frame_id = "/map";
    node_text.header.stamp = ros::Time::now();
    node_text.action = visualization_msgs::Marker::ADD;
    node_text.ns = "karto";
    node_text.id = 0;
    node_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    node_text.scale.z = 0.3;
    node_text.color.a = 1.0;
    node_text.color.r = 1.0;
    node_text.color.g = 1.0;
    node_text.color.b = 1.0;
   
    if(!rba_.get_rba_state().keyframes.empty())
    {
      // Use a spanning tree to estimate the global pose of every node
      //  starting (root) at the given keyframe:
      
      srba_t::frameid2pose_map_t  spantree;
      TKeyFrameID root_keyframe(curr_kf_id_-1);
      rba_.create_complete_spanning_tree(root_keyframe,spantree, 3);

      int id = 0;
      for (srba_t::frameid2pose_map_t::const_iterator itP = spantree.begin();itP!=spantree.end();++itP)
      {
        if (root_keyframe==itP->first) continue;

        const CPose3D p = itP->second.pose;
        
        // Add the vertex to the marker array 
        m.id = id;
        m.pose.position.x = p.x();
        m.pose.position.y = p.y();
        marray.markers.push_back(visualization_msgs::Marker(m));
        id++;

        node_text.id = id;
        node_text.text= boost::to_string(itP->first);
        node_text.pose.position.x = p.x()+0.15; 
        node_text.pose.position.y = p.y()+0.15; 
        marray.markers.push_back(visualization_msgs::Marker(node_text));
        id++;
      }
      
      for (srba_t::rba_problem_state_t::k2k_edges_deque_t::const_iterator itEdge = rba_.get_rba_state().k2k_edges.begin();
          itEdge!=rba_.get_rba_state().k2k_edges.end();++itEdge)
      {
        CPose3D p1, p2;
        if(itEdge->from != root_keyframe)
        {
          srba_t::frameid2pose_map_t::const_iterator itN1 = spantree.find(itEdge->from);
          if(itN1==spantree.end())
            continue;
          p1 = itN1->second.pose;
        }
        if(itEdge->to != root_keyframe)
        {
          srba_t::frameid2pose_map_t::const_iterator itN2 = spantree.find(itEdge->to);
          if(itN2==spantree.end())
            continue;
          p2 = itN2->second.pose;
        }
        geometry_msgs::Point pt1, pt2;
        pt1.x = p1.x();
        pt1.y = p1.y();
        pt2.x = p2.x();
        pt2.y = p2.y();
  
        loop_edge.points.clear();
        loop_edge.points.push_back(pt1);
        loop_edge.points.push_back(pt2);
        loop_edge.id = id;
        marray.markers.push_back(visualization_msgs::Marker(loop_edge));
        id++;
      }

      // Render landmark as pose constraint
      // For each KF: check all its "observations"
      for (srba_t::frameid2pose_map_t::const_iterator it=spantree.begin();it!=spantree.end();++it)
      {
        const TKeyFrameID kf_id = it->first;
        const srba_t::pose_flag_t & pf = it->second;

        const typename srba_t::keyframe_info &kfi = rba_.get_rba_state().keyframes[kf_id];

        for (size_t i=0;i<kfi.adjacent_k2f_edges.size();i++)
        {
          const srba_t::k2f_edge_t * k2f = kfi.adjacent_k2f_edges[i];
          const TKeyFrameID other_kf_id = k2f->feat_rel_pos->id_frame_base;
          if (kf_id==other_kf_id)
            continue; // It's not an constraint with ANOTHER keyframe

          // Is the other KF in the spanning tree?
          srba_t::frameid2pose_map_t::const_iterator other_it=spantree.find(other_kf_id);
          if (other_it==spantree.end()) continue;

          const srba_t::pose_flag_t & other_pf = other_it->second;

          // Add edge between the two KFs to represent the pose constraint:
          mrpt::poses::CPose3D p1 = mrpt::poses::CPose3D(pf.pose);  // Convert to 3D
          mrpt::poses::CPose3D p2 = mrpt::poses::CPose3D(other_pf.pose);

          geometry_msgs::Point pt1, pt2;
          pt1.x = p1.x();
          pt1.y = p1.y();
          pt2.x = p2.x();
          pt2.y = p2.y();
  
          edge.points.clear();
          edge.points.push_back(pt1);
          edge.points.push_back(pt2);
          edge.id = id;
          marray.markers.push_back(visualization_msgs::Marker(edge));
          id++;
        }

    } // end for each KF
  




 /* 
      mrpt::gui::CDisplayWindow3D win("RBA results",640,480);

      // Do nothing
      //     // --------------------------------------------------------------------------------
      // Show 3D view of the resulting map:
      // --------------------------------------------------------------------------------
      srba_t::TOpenGLRepresentationOptions  opengl_options;
      mrpt::opengl::CSetOfObjectsPtr rba_3d = mrpt::opengl::CSetOfObjects::Create();

      rba_.build_opengl_representation(
        curr_kf_id_,  // Root KF: the current (latest) KF
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
    //win.waitForKey();*/
  }
  }
  void SRBASolver::getGraph(std::vector<float> &g)
  {
  }

  void SRBASolver::Clear()
  {
  }

}   //namespace karto_plugins
