#可操作抓取对象列表
manipulation_msgs/GraspableObject[] graspable_objects

#Information required for visualization

sensor_msgs/Image image
sensor_msgs/CameraInfo camera_info

#Holds a single mesh for each recognized graspable object, an empty mesh otherwise
shape_msgs/Mesh[] meshes

#pose to transform the frame of the clusters/object poses into camera coordinates
geometry_msgs/Pose reference_to_camera
