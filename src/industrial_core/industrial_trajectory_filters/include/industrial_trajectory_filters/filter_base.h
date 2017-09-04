/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FILTER_BASE_H_
#define FILTER_BASE_H_

#include <typeinfo>
#include "ros/assert.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.h>

namespace industrial_trajectory_filters
{

/**
 * @brief Message Adapter structure serves as a stand-in for the arm navigation message type with
 * which the filter was specialized.
 *
 */
struct MessageAdapter
{

  /**
   * @brief Arm navigation message type for trajectory filter
   *
   */
  struct Request
  {
    trajectory_msgs::JointTrajectory trajectory;
  } request;
};

/**
 * @brief This version of the FilterBase<T> can be used to encapsulate the functionality from an arm navigation
 *   trajectory filter into a PlanningRequestAdapter that is used by MoveIt.
 *
 * 	 Since this version of the FilterBase<T> class only provides some degree of backwards compatibility
 * 	 with the original FilterBase interface, then some parts of the arm navigation filter must be modified in order
 * 	 to accommodate the differences introduced by this new interface class.
 *
 * 	 The purely virtual methods "update" and "configure" have been preserved since it is assumed that any
 * 	 trajectory filter specialization implemented at least these two methods.  The getter methods for the
 * 	 filter's type and name have been kept as well, however the corresponding members aren't populated from within
 * 	 the scope of the FilterBase<T> class. Thus, the users implementation must fill these members with
 * 	 the appropriate string data.
 *
 * 	 All the "getParam" methods for parameter lookup have been removed and so the filter implementation
 * 	 must be modified to read parameters using the regular roscpp parameter API (either through the node
 * 	 handle or the bare version)
 *
 * Steps for conversion to a Moveit PlanningRequestAdapter:
 *
 *   a - Remove all arm_navigation header files since these can not be included in a catkin based package.
 *
 *   b - Remove all header files that belong to a rosbuild only package.
 *
 *   c - Include the #include <industrial_trajectory_filters/filter_base.h>" header file in your filter's
 *   	 implementation header file.
 *
 *   d-  In your filter implementation, replace the base class "filters::FilterBase<T>" with
 *   	 "industrial_trajectory_filters::FilterBase<T>"
 *
 *   e-  Remove all calls to the getParam() methods from the old FilterBase<T> class and use the roscpp
 *   	 parameter interface for any parameter lookup tasks instead.
 *
 *   f-  In your filter's header file, declare a typedef that specializes your filter template implementation
 *   	 to a version that uses the "MessageAdapter" structure. For instance, if we are converting the NPointFilter
 *   	 filter class then we would add the following after the class declaration:
 *
 *   	 typedef NPointFilter<MessageAdapter> NPointFilterAdapter;
 *
 *   	 In case the "MessageAdapter" structure does not fit the expected template class then the filter's implementation
 *   	 must provide its own adapter structure that resembles the necessary parts of expected arm_navigation message type.
 *   	 The specialization of such filter would then pass the custom adapter structure in the template argument as follows:
 *
 *   	 typedef NPointFilter<CustomMessageAdapter> NPointFilterAdapter;
 *
 *   g-  In your filter's source file, remove the plugin declaration "PLUGINLIB_DECLARE_CLASS" and add the
 *   	 class loader macro.  In the case of the "NPointFilter" this would be as follows:
 *
 *   	 CLASS_LOADER_REGISTER_CLASS(industrial_trajectory_filters::NPointFilterAdapter,
 *		 planning_request_adapter::PlanningRequestAdapter);
 *
 *	 h - All parameters used by your filter must me made available as a regular ros parameter.  For instance, in the
 *	     "ompl_planning_pipeline.launch" file in a moveit package for your robot you would add the following element:
 *	     <param name="my_filter_parameter" value="5" />
 *
 *	 i - (Optional)  the "adaptAndPlan" methods default implementation already maps the trajectory data between the
 *	 	 MessageAdapter structure and the planning interface objects in the argument list.  The filter's implementation
 *	 	 should override this method whenever a custom adapter structure is used.
 */
template<typename T>
  class FilterBase : public planning_request_adapter::PlanningRequestAdapter
  {
  public:

    /**
     * @brief Default constructor
     */
    FilterBase() :
        planning_request_adapter::PlanningRequestAdapter(), nh_("~"), configured_(false), filter_type_("FilterBase"), filter_name_(
            "Unimplemented")
    {

    }

    /**
     * Default destructor
     */
    virtual ~FilterBase()
    {
    }
    ;

    /**
     * @brief Update the filter and return the data separately. This function
     * must be implemented in the derived class.
     *
     * @param data_in A reference to the data to be input to the filter
     * @param data_out A reference to the data output location
     * @return true on success, otherwise false.
     */
    virtual bool update(const T& data_in, T& data_out)=0;

    /**
     * @brief Original FilterBase method, return filter type
     * @return filter type (as string)
     */
    std::string getType()
    {
      return filter_type_;
    }
    ;

    /**
     * @brief Original FitlerBase Method
     * @return filter name (as string).
     */
    inline const std::string& getName()
    {
      return filter_name_;
    }
    ; // original FilterBase method

  protected:

    /**
     * @brief FilterBase method for the sub class to configure the filter
     * This function must be implemented in the derived class.
     * @return true if successful, otherwise false.
     */
    virtual bool configure()=0;

    /**
     * @brief  filter name
     */
    std::string filter_name_;

    /**
     * The type of the filter (Used by FilterChain for Factory construction)
     */
    std::string filter_type_;

    /**
     * @brief Holds filter configuration state.
     */
    bool configured_;

    /**
     * @brief Internal node handle (used for parameter lookup)
     */
    ros::NodeHandle nh_;

  protected:

    /**
     *  @brief Moveit Planning Request Adapter method.  This basic implementation of the
     *  adaptAndPlan method calls the planner and then maps the trajectory data from the
     *  MotionPlanResponse object into the MessageAdapter mapping structure. The
     *  MessageAdapter object is then passed to the "update" method that resembles that
     *  from the old FilterBase interface class.  The filtered trajectory is finally
     *  saved in the MotionPlanResponse object.
     */
    virtual bool adaptAndPlan(const PlannerFn &planner, const planning_scene::PlanningSceneConstPtr &planning_scene,
                              const planning_interface::MotionPlanRequest &req,
                              planning_interface::MotionPlanResponse &res,
                              std::vector<std::size_t> &added_path_index) const
    {

      // non const pointer to this
      FilterBase<MessageAdapter> *p = const_cast<FilterBase<MessageAdapter>*>(this);

      // calling the configure method
      if (!configured_ && p->configure())
      {
        p->configured_ = true;
      }

      // moveit messages for saving trajectory data
      moveit_msgs::RobotTrajectory robot_trajectory_in, robot_trajectory_out;
      MessageAdapter trajectory_in, trajectory_out; // mapping structures

      // calling planner first
      bool result = planner(planning_scene, req, res);

      // applying filter to planned trajectory
      if (result && res.trajectory_)
      {
        // mapping arguments into message adapter struct
        res.trajectory_->getRobotTrajectoryMsg(robot_trajectory_in);
        trajectory_in.request.trajectory = robot_trajectory_in.joint_trajectory;

        // applying arm navigation filter to planned trajectory
        p->update(trajectory_in, trajectory_out);

        // saving filtered trajectory into moveit message.
        robot_trajectory_out.joint_trajectory = trajectory_out.request.trajectory;
        res.trajectory_->setRobotTrajectoryMsg(planning_scene->getCurrentState(), robot_trajectory_out);

      }

      return result;
    }

    /**
     * @brief Return description string
     * @return description (as a string)
     */
    virtual std::string getDescription() const
    {
      // non const pointer to this
      FilterBase<MessageAdapter> *p = const_cast<FilterBase<MessageAdapter>*>(this);

      std::stringstream ss;
      ss << "Trajectory filter '" << p->getName() << "' of type '" << p->getType() << "'";
      return ss.str();
    }
  };

}

#endif /* FILTER_BASE_H_ */
