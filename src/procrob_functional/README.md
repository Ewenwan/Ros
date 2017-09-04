### Copyright 2012-2014 Pouyan Ziafati, University of Luxembourg 
> * All image processing and face recognition functionalities are provided by utilizing the Shervin Emami's c++ source code for face recognition (http://www.shervinemami.info/faceRecognition.html).
 * License: Attribution-NonCommercial 3.0 Unported (http://creativecommons.org/licenses/by-nc/3.0/) 


## Welcome to face\_recognition package for ROS

The **face\_recognition** ROS package provides an simple actionlib server interface for performing different face recognition functionalities in video stream.


## Instalation
This instalation process is for **catkin** (ROS Groovy or newer version)
Assuming that your catkin workspace is under **~/catkin_ws**, if not replace **~/catkin_ws** with appropriate location. It also assumes you're running Bash shell, if you're running Zsh, source appropriate **setup.zsh** file.
```
cd ~/catkin_ws/src
git clone https://github.com/procrob/procrob_functional.git --branch catkin
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
### FaceRecognitionGoal 
This message includes 2 fields: 

* _int_ **order\_id**
* _string_ **order_argument**

### The face recognition actionlib server accepts 4 different goals:
* **order\_id** = **0**
    * **(Recognize Once)** Goal is to acknowledge the first face recognized in the video stream. When the first face is recognized with a confidence value higher than the desirable confidence threshold, the name of the person and the confidence value are sent back to the client as result.

* **order\_id** = **1**
    * **(Recognize Continuously)** Goal is to continuously recognize faces in the video stream. Every face recognized with confidence value higher than the desirable confidence threshold and its confidence value are sent back to the client as feedback. This goal is persuaded for infinite time until it is canceled or preempted by another goal.

* **order\_id** = **2** **and** **order\_argument** = **person\_name**
    * **(Add face images)** Goal is to acquire training images for a NEW person. The video stream is processed for detecting a face which is saved and used as a training image  for the new person. This process is continued until the desired number of training images for the new person is acquired. The name of the new person is provided as "order\_argument"

* **order\_id = 3**
    * **(Train)** The database is (re)trained from the training images

* **order\_id = 4**
    * **(Exit)** The program exits.

### Subscribed Topic:
* **/camera/image\_raw**  -  video stream (standard ROS image transport)

## Parameters:

* **confidence\_value** (_double_, default = 0.88) 
    * a face recognized with confidence value higher than the **confidence\_value** threshold is accepted as valid.
    
* **show\_screen\_flag** (_boolean_, default = true)
    * if output screen is shown

* **add\_face\_number** (int, default = 25)
    * a parameter for the **add\_face\_images** goal which determines the number of training images for a new person to be acquired from the video stream 

## How The Face Recognition Works:

Details in [http://www.shervinemami.info/faceRecognition.html](http://www.shervinemami.info/faceRecognition.html)

The name of persons and the images of their faces to train from are stored in the **train.txt** file.
This file should be in the same directory as where you execute the program. See the example train.txt file for the desirable format of the **train.txt** (Note: the person numbers start from 1, spaces or special characters are not allowed in persons' names). 
The program trains from the **train.txt** file and stores the eigenfaces data in **facedata.xml** file.

When the **face\_recognition** actionlib server is executed (Fserver), the program first tries to load data from the **facedata.xml** file if exists, if not it will try to learn from training images stored in **train.txt** if any. You can always add training data directly from the video stream using the **add\_face\_images** goal (**order\_id** = 2).
Note: when program retrains (**order\_id** = 3), the content of facedata.xml is disregarded and the program trains only based on the train.txt file.

## Tutorial: 
For demonstration purposes an actionlib client example for the face\_recognition simple actionlib server has been provided. 
The client subscribes to **face\_recognition/FRClientGoal** messages. Each FRClientGoal message contains an **order\_id** and an **order\_argument** which specify a goal to be executed by the **face\_recognition** server. After receiving a message, the client sends the corresponding goal to the server. By registering relevant call back functions, the client receives feedback and result information from the execution of goals in the server and prints such information on the terminal. 


* Launch terminal
    * run `roscore`
    * In separate terminal publish a video stream on topic **/camera/image\_raw**.
        * For example you can use **usb_cam** to publish images from your web cam as follows:
        * Install [usb_cam](http://wiki.ros.org/usb_cam) package
        * Run `rosrun usb_cam usb_cam_node usb_cam_node/image_raw:=camera/image_raw _image_height:=<usb_cam_height> _image_width:=<usb_cam_width>`
* In separate terminals run the face recognition server and client as follows:
    * `rosrun face_recognition Fserver`
    * `rosrun face_recognition Fclient`
* In another terminal publish messages on topic **/fr\_order** to test different face recognition functionalities as follows: (notice the info which are printed on the client terminal after each command) 
    * `rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 2 "your_name"`
        * to acquire training images for your face: you should try to appear in the video stream!
    * `rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 3 "none"`
        * to retrain and update the database, so that you can be recognized 
    * `rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 "none"`
        * to recognize faces continuously. This would not stop until you preempt or cancel the goal. So lets preempt it by sending the next goal.
    * `rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 2 "your\_friend's\_name"`   
        * add training images for a new person
    * `rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 0 "none"`
        * recognize once
    * `rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 "none"`
        * recognize continuous
    * `rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 4 "none"`           
        * exit
