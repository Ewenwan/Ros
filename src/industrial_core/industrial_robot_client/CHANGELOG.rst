^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_robot_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2017-01-15)
------------------
* robot_status: missing reply to SERVICE_REQUEST. Fix in robot_status_message and relay_handler.
* industrial_robot_client: Fix signature of goal and cancel callbacks.
* Contributors: Alberto Marini, Maarten de Vries

0.5.0 (2016-02-22)
------------------
* Start checking for trajectory completion as soon as the first moving status arrives
* Forces the action to wait for half of the duration of the trajectory
  before it begins to check for completition. This prevents the action
  for returning immediately for trajectories that end near the start
  point and are slow to start moving.
* Merge pull request `#115 <https://github.com/shaun-edwards/industrial_core/issues/115>`_ from `Jmeyer1292/issue#114 <https://github.com/Jmeyer1292/issue/issues/114>`__start_tolerance_check
  Issue`#114 <https://github.com/shaun-edwards/industrial_core/issues/114>`_ - Action Server withinGoalConstraints check removal
* Revert "Revert "Merge pull request `#113 <https://github.com/shaun-edwards/industrial_core/issues/113>`_ from simonschmeisser/indigo""
  This reverts commit fb28e26fdbd3c316941d4d66af2f1c9b5410cb0d.
* Removed withinTolerance check from the accept function
* Contributors: Jonathan Meyer, Shaun Edwards, Simon Schmeisser

0.4.3 (2016-02-07)
------------------
* Start checking for trajectory completion as soon as the first moving status arrives
* Contributors: Simon Schmeisser

0.4.2 (2015-10-21)
------------------
* Forces the action to wait for half of the duration of the trajectory
  before it begins to check for completition. This prevents the action
  for returning immediately for trajectories that end near the start
  point and are slow to start moving.
* Fix rejected actions when goal requests arrive in between watchdog reset und arrival of next status package
* Contributors: Jonathan Meyer, Shaun Edwards, Simon Schmeisser (isys vision)

0.4.1 (2015-03-23)
------------------
* Fixed changelog links to point to main repo
* Contributors: Shaun Edwards

0.4.0 (2015-03-21)
------------------
* Fill stamp of the RobotStatus message Fix: `#97 <https://github.com/ros-industrial/industrial_core/issues/97>`_
  Just edited on a github didn't test it.
* Only accept goals after reception of controller feedback. Fix `#85 <https://github.com/ros-industrial/industrial_core/issues/85>`_.
* robot_client: workaround for `#46 <https://github.com/ros-industrial/industrial_core/issues/46>`_. Fix `#67 <https://github.com/ros-industrial/industrial_core/issues/67>`_.
  This is an updated version of the workaround committed in 9df46977. Instead
  of requiring dependent packages to invoke the function defined in the
  CFG_EXTRAS cmake snippet, the snippet now sets up the linker path directly.
  Dependent packages now only need to remember to explicitly list their
  dependency on `industrial_robot_client` and `simple_message` in their
  `add_library(..)` statements.
* Contributors: Libor Wagner, gavanderhoorn

0.3.4 (2014-01-21)
------------------
* robot_client: workaround for `#46 <https://github.com/ros-industrial/industrial_core/issues/46>`_. Fix `#67 <https://github.com/ros-industrial/industrial_core/issues/67>`_.
  This is an updated version of the workaround committed in 9df46977. Instead
  of requiring dependent packages to invoke the function defined in the
  CFG_EXTRAS cmake snippet, the snippet now sets up the linker path directly.
  Dependent packages now only need to remember to explicitly list their
  dependency on `industrial_robot_client` and `simple_message` in their
  `add_library(..)` statements.
* Contributors: gavanderhoorn

0.3.3 (2014-01-13)
------------------
* Fixed build issue due simple message library linking
* Contributors: gavanderhoorn

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)

0.3.1 (2014-01-09)
------------------
* Remove obsolete export tags. Fix `#43 <https://github.com/ros-industrial/industrial_core/issues/43>`_.
* Removed library export from catkin macro.  Packages that depend on these must declare library dependencies explicitly (by name)
* Converted to catkin
* Contributors: JeremyZoss, Shaun Edwards, gavanderhoorn
