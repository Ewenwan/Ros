#!/usr/bin/env python

import os
import ycm_core

flags = [
'-Wall',
'-Wextra',
'-Werror',
'-fexceptions',
'-DNDEBUG',
'-std=c++11',
'-x',
'c++',
'-isystem',
'/usr/include',
'-isystem',
'/usr/local/include',
'-isystem',
'/opt/ros/' + os.getenv('ROS_DISTRO') + '/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/devel/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/CMakeLists.txt/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/UArmForROS/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/actionlib_lutorials/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/agitros/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/cv_camera/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/find_object_2d/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/industrial_core/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/learn_gazebo/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/learn_nav_msg/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/learning_tf/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/ork_tutorials/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/procrob_functional/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/rbx1/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/robot_setup_tf/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/ros_arduino_bridge/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/ros_orb/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/rosserial/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/rostensorflow/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/usb_cam/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/using_markers_rviz/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/uvc_cam/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/vision_system/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/voice/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/voice_system/include',
'-isystem',
'/home/ewenwan/ewenwan/catkin_ws/src/目录结构/include'
]

compilation_database_folder = ''

if os.path.exists( compilation_database_folder ):
  database = ycm_core.CompilationDatabase( compilation_database_folder )
else:
  database = None

SOURCE_EXTENSIONS = [ '.cpp', '.cxx', '.cc', '.c' ]

def DirectoryOfThisScript():
  return os.path.dirname( os.path.abspath( __file__ ) )


def MakeRelativePathsInFlagsAbsolute( flags, working_directory ):
  if not working_directory:
    return list( flags )
  new_flags = []
  make_next_absolute = False
  path_flags = [ '-isystem', '-I', '-iquote', '--sysroot=' ]
  for flag in flags:
    new_flag = flag

    if make_next_absolute:
      make_next_absolute = False
      if not flag.startswith( '/' ):
        new_flag = os.path.join( working_directory, flag )

    for path_flag in path_flags:
      if flag == path_flag:
        make_next_absolute = True
        break

      if flag.startswith( path_flag ):
        path = flag[ len( path_flag ): ]
        new_flag = path_flag + os.path.join( working_directory, path )
        break

    if new_flag:
      new_flags.append( new_flag )
  return new_flags


def IsHeaderFile( filename ):
  extension = os.path.splitext( filename )[ 1 ]
  return extension in [ '.h', '.hxx', '.hpp', '.hh' ]


def GetCompilationInfoForFile( filename ):
  if IsHeaderFile( filename ):
    basename = os.path.splitext( filename )[ 0 ]
    for extension in SOURCE_EXTENSIONS:
      replacement_file = basename + extension
      if os.path.exists( replacement_file ):
        compilation_info = database.GetCompilationInfoForFile(
          replacement_file )
        if compilation_info.compiler_flags_:
          return compilation_info
    return None
  return database.GetCompilationInfoForFile( filename )


def FlagsForFile( filename, **kwargs ):
  if database:
    compilation_info = GetCompilationInfoForFile( filename )
    if not compilation_info:
      return None

    final_flags = MakeRelativePathsInFlagsAbsolute(
      compilation_info.compiler_flags_,
      compilation_info.compiler_working_dir_ )
  else:
    relative_to = DirectoryOfThisScript()
    final_flags = MakeRelativePathsInFlagsAbsolute( flags, relative_to )

  return {
    'flags': final_flags,
    'do_cache': True
  }
