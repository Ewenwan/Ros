^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_robot_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2017-01-15)
------------------
* robot_simulator: clarify err msg when controller_joint_names is missing.
* Make industrial_robot_simulator python3 compatible.
* Contributors: G.A. vd. Hoorn, Maarten de Vries

0.5.0 (2016-02-22)
------------------
* Added industrial_robot_client test depends as a workaround to test failures
* Updated industrial_robot_simulator package.xml to version 2 format
* Contributors: Shaun Edwards

0.4.3 (2016-02-07)
------------------
* No changes

0.4.2 (2015-10-21)
------------------
* No change

0.4.1 (2015-03-23)
------------------
* Fixed changelog links to point to main repo
* Contributors: Shaun Edwards

0.4.0 (2015-03-21)
------------------
* Fixed roslaunch test dependency and build depends for robot simulator package
* Corrected roslaunch test and added rospy depends to industrial_robot_simulator package
* Removed extraneous dependencies.  Re-enable launch test
* robot_simulator: add TU Delft copyright as well.
* robot_simulator: add BSD license header. Fix `#90 <https://github.com/ros-industrial/industrial_core/issues/90>`_.
* robot_simulator: set explicit queue size in Publishers. Fix `#99 <https://github.com/ros-industrial/industrial_core/issues/99>`_.
  Queue size of 1 is most likely sufficient for these topics.
* robot_simulator: add GetRobotInfo svc server.
* robot_simulator: quiet down node (use logdebug()).
* Merge pull request `#88 <https://github.com/ros-industrial/industrial_core/issues/88>`_ from gavanderhoorn/rob_sim_robot_status
  Add RobotStatus publishing to robot simulator node
* robot_simulator: remove redundant load_manifest(). Fix `#63 <https://github.com/ros-industrial/industrial_core/issues/63>`_.
* robot_simulator: update manifest (depend on industrial_msgs).
* robot_simulator: publish RobotStatus msgs as well.
* robot_simulator: make initial joint state configurable. Fix `#73 <https://github.com/ros-industrial/industrial_core/issues/73>`_.
* Contributors: Shaun Edwards, gavanderhoorn

0.3.4 (2014-01-21)
------------------
* No change

0.3.3 (2014-01-13)
------------------
* No change

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)

0.3.1 (2014-01-09)
------------------
* robot_simulator: avoid hardcoded python path. Fix `#53 <https://github.com/ros-industrial/industrial_core/issues/53>`_.
* Remove obsolete export tags. Fix `#43 <https://github.com/ros-industrial/industrial_core/issues/43>`_.
* Add install target for launchfile in sim pkg.
  Fix `#35 <https://github.com/ros-industrial/industrial_core/issues/35>`_.
* Fix issue `#6 <https://github.com/ros-industrial/industrial_core/issues/6>`_ (Install target fails for industrial_robot_simulator): setup.py file not required for python executables, see http://ros.org/wiki/catkin/migrating_from_rosbuild
* bugFix: `#61 <https://github.com/ros-industrial/industrial_core/issues/61>`_ - fix joint-name remapping in industrial_robot_simulator
* Added interpolated motion to MotionControllerSimulator class. Includes addition of interpolate(), and modification of  _motion_worker()
* New rospy.get_param() added to IndustrialRobotSimulatorNode in order to assign motion_update_rate
* Converted to catkin
* Contributors: JeremyZoss, Shaun Edwards, dpsolomon, gavanderhoorn, jrgnicho
