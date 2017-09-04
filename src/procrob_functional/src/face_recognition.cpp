/* Copyright 2012 Pouyan Ziafati, University of Luxembourg and Utrecht University

* A ROS simple actionlib server for face recognition in video stream. It provides different face recognition functionalities such as adding training images directly from the video stream, re-training (updating the database to include new training images), recognizing faces in the video stream, etc.

* All image processing and face recognition functionalities are provided by utilizing the Shervin Emami's c++ source code for face recognition (http://www.shervinemami.info/faceRecognition.html). Face Recognition is performed using Eigenfaces (also called "Principal Component Analysis" or PCA) 

*License: Attribution-NonCommercial 3.0 Unported (http://creativecommons.org/licenses/by-nc/3.0/)
*You are free:
    *to Share — to copy, distribute and transmit the work
    *to Remix — to adapt the work

*Under the following conditions:
    *Attribution — You must attribute the work in the manner specified by the author or licensor (but not in any way that suggests that they endorse you or your use of the work).
    *Noncommercial — You may not use this work for commercial purposes.

*With the understanding that:
    *Waiver — Any of the above conditions can be waived if you get permission from the copyright holder.
    *Public Domain — Where the work or any of its elements is in the public domain under applicable law, that status is in no way affected by the license.
    *Other Rights — In no way are any of the following rights affected by the license:
        *Your fair dealing or fair use rights, or other applicable copyright exceptions and limitations;
        *The author's moral rights;
        *Rights other persons may have either in the work itself or in how the work is used, such as publicity or privacy rights.
    *Notice — For any reuse or distribution, you must make clear to others the license terms of this work.

*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cvaux.h>
#include <cxcore.hpp>
#include <sys/stat.h>
#include <termios.h>
//#include <term.h>
#include <unistd.h>
#include "face_recognition_lib.cpp"

using namespace std;

class FaceRecognition
{
public:
    
  FaceRecognition(std::string name) : 
    frl(),
    it_(nh_),
    pnh_("~"),
    as_(nh_, name, boost::bind(&FaceRecognition::executeCB, this, _1), false)
  {
    //a face recognized with confidence value higher than the confidence_value threshold is accepted as valid.
    pnh_.param<double>("confidence_value", confidence_value, 0.88);
    //if output screen is shown
    pnh_.param<bool>("show_screen_flag", show_screen_flag, true);
    ROS_INFO("show_screen_flag: %s", show_screen_flag ? "true" : "false");
    //a parameter for the "add_face_images" goal which determines the number of training images for a new face (person) to be acquired from the video stream 
    pnh_.param<int>("add_face_number", add_face_number, 25);
    add_face_number = 25;
    //the number of persons in the training file (train.txt)
    person_number=0;
    //starting the actionlib server
    as_.start();
    //if the number of persons in the training file is not equal with the number of persons in the trained database, the database is not updated and user should be notified to retrain the database if new tarining images are to be considered.

    if (show_screen_flag) {
      cvNamedWindow("Input", CV_WINDOW_AUTOSIZE); 	// output screen
      cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2,2,CV_AA);  
      textColor = CV_RGB(0,255,255);	// light blue text
    }
    goal_id_ = -99; 

    if(calcNumTrainingPerson("train.txt")!=frl.nPersons)
    {
       frl.database_updated=false; 
       //ROS_INFO("Alert: Database is not updated, You better (re)train from images!");       //BEFORE
       ROS_WARN("Alert: Database is not updated. Please delete \"facedata.xml\" and re-run!"); //AFTER
    }
    //subscribe to video stream through image transport class
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &FaceRecognition::imageCB, this);
  }

  ~FaceRecognition(void)
  {
    if (show_screen_flag) {
     cvDestroyWindow("Input");
    }
  }


  void executeCB(const face_recognition::FaceRecognitionGoalConstPtr &goal)
  {
    //check to be sure if the goal should be still persuaded
    if( as_.isPreemptRequested() || ros::isShuttingDown() )
    {
      as_.setPreempted(); 
      return;
    }
    //check if the name of the person has been provided for the add-face-images goal  
    if(goal->order_id == 2 && goal->order_argument.empty() ) 
    {
      ROS_INFO("No name has been provided for the add_person_images goal");
      as_.setPreempted();
      return;
    }
    ros::Rate r(4);
    //Storing the information about the current goal and reseting feedback and result variables  
    goal_argument_ = goal->order_argument;
    result_.order_id = goal->order_id;  
    feedback_.order_id = goal->order_id;
    result_.names.clear();
    result_.confidence.clear();
    feedback_.names.clear();
    feedback_.confidence.clear();
    goal_id_ = goal->order_id;
    switch(goal_id_)
    { 
      //(exit) Goal is to exit
      case 4: ROS_INFO("exit request");
              as_.setSucceeded(result_);
              r.sleep();
              ros::shutdown(); 
              break;
      case 3:
          //(train_database) Goal is to (re)train the database from training images
          if(frl.retrainOnline()) 
            as_.setSucceeded(result_);
          else 
            as_.setAborted(result_);
          break;
      //(recognize_once) Goal is to recognize the person in the video stream, succeed when the first person is found
      case 0:
      //(recognize_continuous) Goal is to Continuously recognize persons in the video stream and provide feedback continuously. This goal is persuaded for infinite time
      case 1:
      //(add_face_images) Goal is to take a number of(add_face_number) images of a person's face from the video stream and save them as training images 
      case 2:
         
	 {
	  if(goal_id_ == 2)
            add_face_count=0;
          //to synchronize with processes performed in the subscribed function to the video stream (imageCB)
          //as far as the goal id is 0, 1 or 2, it's active and there is no preempting request, imageCB function can be called.
          while( as_.isActive() && !as_.isPreemptRequested() && !ros::isShuttingDown() )
            r.sleep();
	  mutex_.lock();
	  if(as_.isActive())	
          { 
             as_.setPreempted();
             ROS_INFO("Goal %d is preempted",goal_id_);
          } 
          goal_id_ = -99;
          mutex_.unlock();       
 	  break;	       
	 }
        
    }
    goal_id_ = -99; 
}

  int calcNumTrainingPerson(const char * filename)
  {
      FILE * imgListFile = 0;
      char imgFilename[512];
      int iFace, nFaces=0;
      int person_num=0;
      // open the input file
      if( !(imgListFile = fopen(filename, "r")) )
        {
	   fprintf(stderr, "Can\'t open file %s\n", filename);
	   return 0;
	}
      // count the number of faces
      while( fgets(imgFilename, 512, imgListFile) ) ++nFaces;
      rewind(imgListFile);
      //count the number of persons
      for(iFace=0; iFace<nFaces; iFace++)
      {
	char personName[256];
	int personNumber;
	// read person number (beginning with 1), their name and the image filename.
	fscanf(imgListFile, "%d %s %s", &personNumber, personName, imgFilename);
	if (personNumber > person_num) 
           person_num = personNumber;
      }
      fclose(imgListFile); 
      return (person_num);     
  }
  void imageCB(const sensor_msgs::ImageConstPtr& msg)
  {
    //to synchronize with executeCB function.
    //as far as the goal id is 0, 1 or 2, it's active and there is no preempting request, imageCB function is proceed.
    if (!as_.isActive() || goal_id_ > 2)      return;
    if(!mutex_.try_lock()) return;    
    if(as_.isPreemptRequested())    
    {
       ROS_INFO("Goal %d is preempted",goal_id_);
       as_.setPreempted();
       mutex_.unlock(); return;
    }  

    //get the value of show_screen_flag from the parameter server
    ros::param::getCached("~show_screen_flag", show_screen_flag); 

    cv_bridge::CvImagePtr cv_ptr;
    //convert from ros image format to opencv image format
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      as_.setPreempted();
      ROS_INFO("Goal %d is preempted",goal_id_);
      mutex_.unlock();
      return;
    }
    ros::Rate r(4);   
    IplImage img_input = cv_ptr->image;
    IplImage *img= cvCloneImage(&img_input); 
    IplImage *greyImg;
    IplImage *faceImg;
    IplImage *sizedImg;
    IplImage *equalizedImg;
    CvRect faceRect;
    // Make sure the image is greyscale, since the Eigenfaces is only done on greyscale image.
    greyImg = frl.convertImageToGreyscale(img);
    // Perform face detection on the input image, using the given Haar cascade classifier.
    faceRect = frl.detectFaceInImage(greyImg,frl.faceCascade);
    // Make sure a valid face was detected.
    if (faceRect.width < 1) 
    {
      ROS_INFO("No face was detected in the last frame"); 
      if(show_screen_flag)
      {
        cvPutText(img, text_image.str().c_str(), cvPoint(10, faceRect.y + 50), &font, textColor);
        cvShowImage("Input", img);
        cvWaitKey(1);
      }
      cvReleaseImage( &greyImg );cvReleaseImage(&img);
      r.sleep(); 
      mutex_.unlock(); return;
    }
    if (show_screen_flag) {
      cvRectangle(img, cvPoint(faceRect.x, faceRect.y), cvPoint(faceRect.x + faceRect.width-1, faceRect.y + faceRect.height-1), CV_RGB(0,255,0), 1, 8, 0);
    }
    faceImg = frl.cropImage(greyImg, faceRect);	// Get the detected face image.
    // Make sure the image is the same dimensions as the training images.
    sizedImg = frl.resizeImage(faceImg, frl.faceWidth, frl.faceHeight);
    // Give the image a standard brightness and contrast, in case it was too dark or low contrast.
    equalizedImg = cvCreateImage(cvGetSize(sizedImg), 8, 1);	// Create an empty greyscale image
    cvEqualizeHist(sizedImg, equalizedImg);
    cvReleaseImage( &greyImg );cvReleaseImage( &faceImg );cvReleaseImage( &sizedImg );  
    //check again if preempting request is not there!
    if(as_.isPreemptRequested())    
        {
           ROS_INFO("Goal %d is preempted",goal_id_);
           cvReleaseImage(&equalizedImg);cvReleaseImage(&img);  
           as_.setPreempted(); 
           ROS_INFO("Goal %d is preempted",goal_id_);
           mutex_.unlock(); return;
        }
     //goal is add_face_images
     if( goal_id_==2  )      
     {
        if(add_face_count==0)  
        {
           //assign the correct number for the new person
           person_number = calcNumTrainingPerson("train.txt")+1; 
        }
        char cstr[256];
	sprintf(cstr, "data/%d_%s%d.pgm", person_number, &goal_argument_[0], add_face_count+1);
        ROS_INFO("Storing the current face of '%s' into image '%s'.", &goal_argument_[0], cstr);
        //save the new training image of the person
        cvSaveImage(cstr, equalizedImg, NULL);
	// Append the new person to the end of the training data.
	trainFile = fopen("train.txt", "a");
	fprintf(trainFile, "%d %s %s\n", person_number, &goal_argument_[0], cstr);
	fclose(trainFile);
        if(add_face_count==0)  
        {
           //get from parameter server how many training imaged should be acquire.
           ros::param::getCached("~add_face_number", add_face_number);
           if(add_face_number<=0)
             {
               ROS_INFO("add_face_number parameter is Zero, it is Invalid. One face was added anyway!");
               add_face_number=1;
             } 
           frl.database_updated = false;
        }
        if (show_screen_flag) {
          text_image.str("");
          text_image <<"A picture of "<< &goal_argument_[0]<< "was added" <<endl;
          cvPutText(img, text_image.str().c_str(), cvPoint( 10, 50), &font, textColor);
        }
        //check if enough number of training images for the person has been acquired, then the goal is succeed.
        if(++add_face_count==add_face_number)	
	{
           
           result_.names.push_back(goal_argument_); 
           as_.setSucceeded(result_);
           if (show_screen_flag)
           {
              cvShowImage("Input", img);
              cvWaitKey(1);
           }
           cvReleaseImage(&equalizedImg); cvReleaseImage(&img);
           mutex_.unlock(); return;
        }   
        feedback_.names.clear();
        feedback_.confidence.clear();
        feedback_.names.push_back(goal_argument_);
//      feedback_.confidence.push_back();
        as_.publishFeedback(feedback_);     
     }
     //goal is to recognize person in the video stream
     if( goal_id_<2 )      
    {
       int iNearest, nearest;
       float confidence;
       float * projectedTestFace=0;
       if(!frl.database_updated)
          //ROS_INFO("Alert: Database is not updated, You better (re)train from images!");       //BEFORE
	 ROS_WARN("Alert: Database is not updated. Please delete \"facedata.xml\" and re-run!"); //AFTER
       if(frl.nEigens < 1) 
       {
          ROS_INFO("NO database available, goal is Aborted");
          cvReleaseImage(&equalizedImg);cvReleaseImage(&img);  
          ROS_INFO("Goal %d is Aborted",goal_id_);
          as_.setAborted(); 
          mutex_.unlock(); return;
       }
        // Project the test images onto the PCA subspace
       projectedTestFace = (float *)cvAlloc( frl.nEigens*sizeof(float) );
       // project the test image onto the PCA subspace
       cvEigenDecomposite(equalizedImg,frl.nEigens,frl.eigenVectArr,0, 0,frl.pAvgTrainImg,projectedTestFace);
       // Check which person it is most likely to be.
       iNearest = frl.findNearestNeighbor(projectedTestFace, &confidence);
       nearest  = frl.trainPersonNumMat->data.i[iNearest];
       //get the desired confidence value from the parameter server
       ros::param::getCached("~confidence_value", confidence_value);
       cvFree(&projectedTestFace);
       if(confidence<confidence_value)
       {
          ROS_INFO("Confidence is less than %f was %f, detected face is not considered.",(float)confidence_value, (float)confidence);
          if (show_screen_flag) {
            text_image.str("");
            text_image << "Confidence is less than "<< confidence_value;
            cvPutText(img, text_image.str().c_str(), cvPoint(faceRect.x, faceRect.y + faceRect.height + 25), &font, textColor);
          }
       }
       else
       {
         if (show_screen_flag) {
           text_image.str("");
           text_image <<  frl.personNames[nearest-1].c_str()<<" is recognized";
           cvPutText(img, text_image.str().c_str(), cvPoint(faceRect.x, faceRect.y + faceRect.height + 25), &font, textColor);
         }
	  //goal is to recognize_once, therefore set as succeeded.
          if(goal_id_==0)
          {
             result_.names.push_back(frl.personNames[nearest-1].c_str());
             result_.confidence.push_back(confidence);
             as_.setSucceeded(result_);
          }
          //goal is recognize continuous, provide feedback and continue.
          else
          {
	     ROS_INFO("detected %s  confidence %f ",  frl.personNames[nearest-1].c_str(),confidence);              
             feedback_.names.clear();
             feedback_.confidence.clear();
             feedback_.names.push_back(frl.personNames[nearest-1].c_str());
             feedback_.confidence.push_back(confidence);
             as_.publishFeedback(feedback_);                
          }
                             
       }
    
    }
    if (show_screen_flag)
    {
       cvShowImage("Input", img);
       cvWaitKey(1);
    } 
    cvReleaseImage(&equalizedImg);   cvReleaseImage(&img);
    r.sleep();
    mutex_.unlock();
    return;		
  }


protected:
    
  boost::mutex mutex_; //for synchronization between executeCB and imageCB
  std::string goal_argument_; 
  int goal_id_;
  face_recognition::FaceRecognitionFeedback feedback_; 
  face_recognition::FaceRecognitionResult result_;
  int add_face_count; //help variable to count the number of training images already taken in the add_face_images goal
  FILE *trainFile; 
  double confidence_value;//a face recognized with confidence value higher than confidence_value threshold is accepted as valid.
  bool   show_screen_flag;//if output window is shown
  int    add_face_number; //the number of training images to be taken in add_face_images goal
  CvFont font;
  CvScalar textColor;
  ostringstream text_image;
  ros::NodeHandle nh_, pnh_;
  FaceRecognitionLib frl; 
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  actionlib::SimpleActionServer<face_recognition::FaceRecognitionAction> as_;
  int person_number;     //the number of persons in the train file (train.txt)
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_recognition");
  FaceRecognition face_recognition(ros::this_node::getName());
  ros::spin();
  return 0;
}

