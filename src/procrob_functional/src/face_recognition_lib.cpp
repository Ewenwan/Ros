/* Copyright 2012 Pouyan Ziafati, University of Luxembourg and Utrecht University

*The face_recognition_lib.cpp provides image processing and face recognition functionalities for the face_recognition ROS package. 
* All image processing and face recognition functionalities are provided by utilizing the Shervin Emami's c++ source code for face recognition (http://www.shervinemami.info/faceRecognition.html).


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

//Headers added by D.Portugal (needed for Catkinization):
#include <cvaux.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <ros/package.h> //to get pkg path

using namespace std;
static const char *faceCascadeFilename = "haarcascade_frontalface_alt.xml";   // Haar Cascade file, used for Face Detection.
const std::string path = ros::package::getPath("face_recognition"); 	//D.Portugal => get pkg path
//#define USE_MAHALANOBIS_DISTANCE	// You might get better recognition accuracy if you enable this.
class FaceRecognitionLib
{
public:
  // Global variables
  int SAVE_EIGENFACE_IMAGES;// Set to 0 if you don't want images of the Eigenvectors saved to files (for debugging).
  IplImage ** faceImgArr; // array of face images
  vector<string> personNames;			// array of person names (indexed by the person number). Added by Shervin.
  int faceWidth;	// Default dimensions for faces in the face recognition database. Added by Shervin.
  int faceHeight;	//	"		"		"		"		"		"
  int nPersons; // the number of people in the training set. Added by Shervin.
  int nTrainFaces; // the number of training images
  int nEigens; // the number of eigenvalues
  IplImage * pAvgTrainImg; // the average image
  IplImage ** eigenVectArr; // eigenvectors
  CvMat * eigenValMat; // eigenvalues
  CvMat * projectedTrainFaceMat; // projected training faces
  CvHaarClassifierCascade* faceCascade;
  CvMat * trainPersonNumMat;  // the person numbers during training
  bool database_updated;
  //Functions:
  bool learn(const char *szFileTrain);
  void doPCA();
  void storeTrainingData();
  int  loadTrainingData(CvMat ** pTrainPersonNumMat);
  int  findNearestNeighbor(float * projectedTestFace);
  int  findNearestNeighbor(float * projectedTestFace, float *pConfidence);
  int  loadFaceImgArray(const char * filename);
  void storeEigenfaceImages();
  IplImage* convertImageToGreyscale(const IplImage *imageSrc);
  IplImage* cropImage(const IplImage *img, const CvRect region);
  IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight);
  IplImage* convertFloatImageToUcharImage(const IplImage *srcImg);
  CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade );
  bool retrainOnline(void);
  FaceRecognitionLib()
  {
     SAVE_EIGENFACE_IMAGES = 1;		
     faceImgArr= 0; 
     faceWidth = 120;
     faceHeight =  90;
     nPersons                  = 0;
     nTrainFaces               = 0;
     nEigens                   = 0;
     pAvgTrainImg       = 0;
     eigenVectArr      = 0; 
     eigenValMat           = 0; 
     projectedTrainFaceMat = 0; 
     database_updated = false;
     
     /** D.Portugal: needed in case you "rosrun" from another folder **/    
     chdir(path.c_str());
     
     // Load the HaarCascade classifier for face detection.
     faceCascade = (CvHaarClassifierCascade*)cvLoad(faceCascadeFilename, 0, 0, 0 );
     if( !faceCascade ) 
     {
       ROS_INFO("Could not load Haar cascade Face detection classifier in '%s'.", faceCascadeFilename);
       exit(1);
     }   
     // Make sure there is a "data" folder, for storing the new person.
     mkdir("data",S_IRWXU | S_IRWXG | S_IRWXO); 
     // Load the previously saved training data
     trainPersonNumMat = 0;
     if( loadTrainingData( &trainPersonNumMat ) ) 
     {
	faceWidth = pAvgTrainImg->width;
	faceHeight = pAvgTrainImg->height;
        database_updated=true;
     }
     else
     {
        ROS_INFO("Will try to train from images");
        if(!retrainOnline()) 
           ROS_INFO("Could not train from images");	
	
     }
  }
  ~FaceRecognitionLib(void)
  {
        cvReleaseHaarClassifierCascade( &faceCascade );
        if(trainPersonNumMat)	cvReleaseMat( &trainPersonNumMat );
	int i;
        if (faceImgArr)
        {
	   for (i=0; i<nTrainFaces; i++)
              if (faceImgArr[i])	cvReleaseImage( &faceImgArr[i] );
	   cvFree( &faceImgArr );            	                   
        }
        if(eigenVectArr)
        {
           for (i=0; i<nEigens; i++)
              if (eigenVectArr[i])      cvReleaseImage( &eigenVectArr[i] );
	   cvFree( &eigenVectArr ); 
        }
        if(trainPersonNumMat) cvReleaseMat( &trainPersonNumMat ); 
	personNames.clear();		
	if(pAvgTrainImg) cvReleaseImage( &pAvgTrainImg ); 	
	if(eigenValMat)  cvReleaseMat( &eigenValMat );
	if(projectedTrainFaceMat) cvReleaseMat( &projectedTrainFaceMat );
   }

};


// Perform face detection on the input image, using the given Haar cascade classifier.
// Returns a rectangle for the detected region in the given image.
CvRect FaceRecognitionLib::detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade )
{
	const CvSize minFeatureSize = cvSize(20, 20);
	const int flags = CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH;	// Only search for 1 face.
	const float search_scale_factor = 1.1f;
	IplImage *detectImg;
	IplImage *greyImg = 0;
	CvMemStorage* storage;
	CvRect rc;
	//double t;
	CvSeq* rects;


	storage = cvCreateMemStorage(0);
	cvClearMemStorage( storage );

	// If the image is color, use a greyscale copy of the image.
	detectImg = (IplImage*)inputImg;	// Assume the input image is to be used.
	if (inputImg->nChannels > 1)
	{
		greyImg = cvCreateImage(cvSize(inputImg->width, inputImg->height), IPL_DEPTH_8U, 1 );
		cvCvtColor( inputImg, greyImg, CV_BGR2GRAY );
		detectImg = greyImg;	// Use the greyscale version as the input.
	}

	// Detect all the faces.
	//$$$$$t = (double)cvGetTickCount();
	rects = cvHaarDetectObjects( detectImg, (CvHaarClassifierCascade*)cascade, storage,
				search_scale_factor, 3, flags, minFeatureSize );
	//$$$$$t = (double)cvGetTickCount() - t;
	//$$$$$ROS_INFO("[Face Detection took %d ms and found %d objects]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ), rects->total );

	// Get the first detected face (the biggest).
	if (rects->total > 0)
        {
          rc = *(CvRect*)cvGetSeqElem( rects, 0 );
        }
	else
		rc = cvRect(-1,-1,-1,-1);	// Couldn't find the face.

	//cvReleaseHaarClassifierCascade( &cascade );
	//cvReleaseImage( &detectImg );
	if (greyImg)
	cvReleaseImage( &greyImg );
	cvReleaseMemStorage( &storage );

	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
}



// Return a new image that is always greyscale, whether the input image was RGB or Greyscale.
// Remember to free the returned image using cvReleaseImage() when finished.
IplImage* FaceRecognitionLib::convertImageToGreyscale(const IplImage *imageSrc)
{
	IplImage *imageGrey;
	// Either convert the image to greyscale, or make a copy of the existing greyscale image.
	// This is to make sure that the user can always call cvReleaseImage() on the output, whether it was greyscale or not.
	if (imageSrc->nChannels == 3) {
		imageGrey = cvCreateImage( cvGetSize(imageSrc), IPL_DEPTH_8U, 1 );
		cvCvtColor( imageSrc, imageGrey, CV_BGR2GRAY );
	}
	else {
		imageGrey = cvCloneImage(imageSrc);
	}
	return imageGrey;
}

// Creates a new image copy that is of a desired size.
// Remember to free the new image later.
IplImage* FaceRecognitionLib::resizeImage(const IplImage *origImg, int newWidth, int newHeight)
{
	IplImage *outImg = 0;
	int origWidth;
	int origHeight;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if (newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0) {
		ROS_INFO("ERROR in resizeImage: Bad desired image size of %dx%d.", newWidth, newHeight);
		exit(1);
	}

	// Scale the image to the new dimensions, even if the aspect ratio will be changed.
	outImg = cvCreateImage(cvSize(newWidth, newHeight), origImg->depth, origImg->nChannels);
	if (newWidth > origImg->width && newHeight > origImg->height) {
		// Make the image larger
		cvResetImageROI((IplImage*)origImg);
		cvResize(origImg, outImg, CV_INTER_LINEAR);	// CV_INTER_CUBIC or CV_INTER_LINEAR is good for enlarging
	}
	else {
		// Make the image smaller
		cvResetImageROI((IplImage*)origImg);
		cvResize(origImg, outImg, CV_INTER_AREA);	// CV_INTER_AREA is good for shrinking / decimation, but bad at enlarging.
	}

	return outImg;
}

// Returns a new image that is a cropped version of the original image.
IplImage* FaceRecognitionLib::cropImage(const IplImage *img, const CvRect region)
{
	IplImage *imageTmp;
	IplImage *imageRGB;
	CvSize size;
	size.height = img->height;
	size.width = img->width;

	if (img->depth != IPL_DEPTH_8U) {
		ROS_INFO("ERROR in cropImage: Unknown image depth of %d given in cropImage() instead of 8 bits per pixel.", img->depth);
		exit(1);
	}

	// First create a new (color or greyscale) IPL Image and copy contents of img into it.
	imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(img, imageTmp, NULL);

	// Create a new image of the detected region
	// Set region of interest to that surrounding the face
	cvSetImageROI(imageTmp, region);
	// Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
	size.width = region.width;
	size.height = region.height;
	imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(imageTmp, imageRGB, NULL);	// Copy just the region.

    cvReleaseImage( &imageTmp );
	return imageRGB;
}

// Get an 8-bit equivalent of the 32-bit Float image.
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* FaceRecognitionLib::convertFloatImageToUcharImage(const IplImage *srcImg)
{
	IplImage *dstImg = 0;
	if ((srcImg) && (srcImg->width > 0 && srcImg->height > 0)) {

		// Spread the 32bit floating point pixels to fit within 8bit pixel range.
		double minVal, maxVal;
		cvMinMaxLoc(srcImg, &minVal, &maxVal);

		//cout << "FloatImage:(minV=" << minVal << ", maxV=" << maxVal << ")." << endl;

		// Deal with NaN and extreme values, since the DFT seems to give some NaN results.
		if (cvIsNaN(minVal) || minVal < -1e30)
			minVal = -1e30;
		if (cvIsNaN(maxVal) || maxVal > 1e30)
			maxVal = 1e30;
		if (maxVal-minVal == 0.0f)
			maxVal = minVal + 0.001;	// remove potential divide by zero errors.

		// Convert the format
		dstImg = cvCreateImage(cvSize(srcImg->width, srcImg->height), 8, 1);
		cvConvertScale(srcImg, dstImg, 255.0 / (maxVal - minVal), - minVal * 255.0 / (maxVal-minVal));
	}
	return dstImg;
}




void FaceRecognitionLib::storeEigenfaceImages()
{
	// Store the average image to a file
	ROS_INFO("Saving the image of the average face as 'out_averageImage.bmp'.");
	cvSaveImage("out_averageImage.bmp", pAvgTrainImg);
	// Create a large image made of many eigenface images.
	// Must also convert each eigenface image to a normal 8-bit UCHAR image instead of a 32-bit float image.
	ROS_INFO("Saving the %d eigenvector images as 'out_eigenfaces.bmp'", nEigens);
	if (nEigens > 0) {
		// Put all the eigenfaces next to each other.
		int COLUMNS = 8;	// Put upto 8 images on a row.
		int nCols = min(nEigens, COLUMNS);
		int nRows = 1 + (nEigens / COLUMNS);	// Put the rest on new rows.
		int w = eigenVectArr[0]->width;
		int h = eigenVectArr[0]->height;
		CvSize size;
		size = cvSize(nCols * w, nRows * h);
		IplImage *bigImg = cvCreateImage(size, IPL_DEPTH_8U, 1);	// 8-bit Greyscale UCHAR image
		for (int i=0; i<nEigens; i++) {
			// Get the eigenface image.
			IplImage *byteImg = convertFloatImageToUcharImage(eigenVectArr[i]);
			// Paste it into the correct position.
			int x = w * (i % COLUMNS);
			int y = h * (i / COLUMNS);
			CvRect ROI = cvRect(x, y, w, h);
			cvSetImageROI(bigImg, ROI);
			cvCopyImage(byteImg, bigImg);
			cvResetImageROI(bigImg);
			cvReleaseImage(&byteImg);
		}
		cvSaveImage("out_eigenfaces.bmp", bigImg);
		cvReleaseImage(&bigImg);
	}
}

// Train from the data in the given text file, and store the trained data into the file 'facedata.xml'.
bool FaceRecognitionLib::learn(const char *szFileTrain)
{
	int i, offset;

	// load training data
	ROS_INFO("Loading the training images in '%s'", szFileTrain);
	nTrainFaces = loadFaceImgArray(szFileTrain);
	ROS_INFO("Got %d training images.\n", nTrainFaces);
	if( nTrainFaces < 2 )
	{
		fprintf(stderr,
		        "Need 2 or more training faces"
		        "Input file contains only %d", nTrainFaces);
		return false;
	}

	// do PCA on the training faces
	doPCA();

	// project the training images onto the PCA subspace
	projectedTrainFaceMat = cvCreateMat( nTrainFaces, nEigens, CV_32FC1 );
	offset = projectedTrainFaceMat->step / sizeof(float);
	for(i=0; i<nTrainFaces; i++)
	{
		//int offset = i * nEigens;
		cvEigenDecomposite(
			faceImgArr[i],
			nEigens,
			eigenVectArr,
			0, 0,
			pAvgTrainImg,
			//projectedTrainFaceMat->data.fl + i*nEigens);
			projectedTrainFaceMat->data.fl + i*offset);
	}

	// store the recognition data as an xml file
	storeTrainingData();

	// Save all the eigenvectors as images, so that they can be checked.
	//if (SAVE_EIGENFACE_IMAGES) {
	//	storeEigenfaceImages();
	//}
        return true;

}


// Open the training data from the file 'facedata.xml'.
int FaceRecognitionLib::loadTrainingData(CvMat ** pTrainPersonNumMat)
{
	CvFileStorage * fileStorage;
	int i;

	// create a file-storage interface
	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_READ );
	if( !fileStorage ) {
		ROS_INFO("Can't open training database file 'facedata.xml'.");
		return 0;
	}

	// Load the person names. Added by Shervin.
	personNames.clear();	// Make sure it starts as empty.
	nPersons = cvReadIntByName( fileStorage, 0, "nPersons", 0 );
	if (nPersons == 0) {
		ROS_INFO("No people found in the training database 'facedata.xml'.");
		return 0;
	}
	// Load each person's name.
	for (i=0; i<nPersons; i++) {
		string sPersonName;
		char varname[200];
		sprintf( varname, "personName_%d", (i+1) );
		sPersonName = cvReadStringByName(fileStorage, 0, varname );
		personNames.push_back( sPersonName );
	}

	// Load the data
	nEigens = cvReadIntByName(fileStorage, 0, "nEigens", 0);
	nTrainFaces = cvReadIntByName(fileStorage, 0, "nTrainFaces", 0);
	*pTrainPersonNumMat = (CvMat *)cvReadByName(fileStorage, 0, "trainPersonNumMat", 0);
	eigenValMat  = (CvMat *)cvReadByName(fileStorage, 0, "eigenValMat", 0);
	projectedTrainFaceMat = (CvMat *)cvReadByName(fileStorage, 0, "projectedTrainFaceMat", 0);
	pAvgTrainImg = (IplImage *)cvReadByName(fileStorage, 0, "avgTrainImg", 0);
	eigenVectArr = (IplImage **)cvAlloc(nTrainFaces*sizeof(IplImage *));
	for(i=0; i<nEigens; i++)
	{
		char varname[200];
		sprintf( varname, "eigenVect_%d", i );
		eigenVectArr[i] = (IplImage *)cvReadByName(fileStorage, 0, varname, 0);
	}

	// release the file-storage interface
	cvReleaseFileStorage( &fileStorage );

	ROS_INFO("Training data loaded (%d training images of %d people):", nTrainFaces, nPersons);
	ROS_INFO("People: ");
	if (nPersons > 0)
		ROS_INFO("<%s>", personNames[0].c_str());
	for (i=1; i<nPersons; i++) {
		ROS_INFO(", <%s>", personNames[i].c_str());
	}
        database_updated = true;
	return 1;
}


// Save the training data to the file 'facedata.xml'.
void FaceRecognitionLib::storeTrainingData()
{
	CvFileStorage * fileStorage;
	int i;

	// create a file-storage interface
	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_WRITE );

	// Store the person names. Added by Shervin.
	cvWriteInt( fileStorage, "nPersons", nPersons );
	for (i=0; i<nPersons; i++) {
		char varname[200];
		sprintf( varname, "personName_%d", (i+1) );
		cvWriteString(fileStorage, varname, personNames[i].c_str(), 0);
	}

	// store all the data
	cvWriteInt( fileStorage, "nEigens", nEigens );
	cvWriteInt( fileStorage, "nTrainFaces", nTrainFaces );
	cvWrite(fileStorage, "trainPersonNumMat", trainPersonNumMat, cvAttrList(0,0));
	cvWrite(fileStorage, "eigenValMat", eigenValMat, cvAttrList(0,0));
	cvWrite(fileStorage, "projectedTrainFaceMat", projectedTrainFaceMat, cvAttrList(0,0));
	cvWrite(fileStorage, "avgTrainImg", pAvgTrainImg, cvAttrList(0,0));
	for(i=0; i<nEigens; i++)
	{
		char varname[200];
		sprintf( varname, "eigenVect_%d", i );
		cvWrite(fileStorage, varname, eigenVectArr[i], cvAttrList(0,0));
	}

	// release the file-storage interface
	cvReleaseFileStorage( &fileStorage );
}

// Find the most likely person based on a detection. Returns the index, and stores the confidence value into pConfidence.
int FaceRecognitionLib::findNearestNeighbor(float * projectedTestFace, float *pConfidence)
{
	//double leastDistSq = 1e12;
	double leastDistSq = DBL_MAX;
	int i, iTrain, iNearest = 0;

	for(iTrain=0; iTrain<nTrainFaces; iTrain++)
	{
		double distSq=0;

		for(i=0; i<nEigens; i++)
		{
			float d_i = projectedTestFace[i] - projectedTrainFaceMat->data.fl[iTrain*nEigens + i];
#ifdef USE_MAHALANOBIS_DISTANCE
			distSq += d_i*d_i / eigenValMat->data.fl[i];  // Mahalanobis distance (might give better results than Eucalidean distance)
#else
			distSq += d_i*d_i; // Euclidean distance.
#endif
		}

		if(distSq < leastDistSq)
		{
			leastDistSq = distSq;
			iNearest = iTrain;
		}
	}

	// Return the confidence level based on the Euclidean distance,
	// so that similar images should give a confidence between 0.5 to 1.0,
	// and very different images should give a confidence between 0.0 to 0.5.
	*pConfidence = 1.0f - sqrt( leastDistSq / (float)(nTrainFaces * nEigens) ) / 255.0f;

	// Return the found index.
	return iNearest;
}

// Do the Principal Component Analysis, finding the average image
// and the eigenfaces that represent any image in the given dataset.
void FaceRecognitionLib::doPCA()
{
	int i;
	CvTermCriteria calcLimit;
	CvSize faceImgSize;

	// set the number of eigenvalues to use
	nEigens = nTrainFaces-1;

	// allocate the eigenvector images
	faceImgSize.width  = faceImgArr[0]->width;
	faceImgSize.height = faceImgArr[0]->height;
	eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);
	for(i=0; i<nEigens; i++)
		eigenVectArr[i] = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

	// allocate the eigenvalue array
	eigenValMat = cvCreateMat( 1, nEigens, CV_32FC1 );

	// allocate the averaged image
	pAvgTrainImg = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

	// set the PCA termination criterion
	calcLimit = cvTermCriteria( CV_TERMCRIT_ITER, nEigens, 1);
        ROS_INFO("**** nTrainFaces: %d",nTrainFaces);
	// compute average image, eigenvalues, and eigenvectors
	cvCalcEigenObjects(
		nTrainFaces,
		(void*)faceImgArr,
		(void*)eigenVectArr,
		CV_EIGOBJ_NO_CALLBACK,
		0,
		0,
		&calcLimit,
		pAvgTrainImg,
		eigenValMat->data.fl);

	cvNormalize(eigenValMat, eigenValMat, 1, 0, CV_L1, 0);
}

// Read the names & image filenames of people from a text file, and load all those images listed.
int FaceRecognitionLib::loadFaceImgArray(const char * filename)
{
	FILE * imgListFile = 0;
	char imgFilename[512];
	int iFace, nFaces=0;
	int i;
        IplImage *pfaceImg;
        IplImage *psizedImg;
        IplImage *pequalizedImg;
	// open the input file
	if( !(imgListFile = fopen(filename, "r")) )
	{
		fprintf(stderr, "Can\'t open file %s\n", filename);
		return 0;
	}

	// count the number of faces
	while( fgets(imgFilename, 512, imgListFile) ) ++nFaces;
	rewind(imgListFile);

	// allocate the face-image array and person number matrix
	faceImgArr        = (IplImage **)cvAlloc( nFaces*sizeof(IplImage *) );
	trainPersonNumMat = cvCreateMat( 1, nFaces, CV_32SC1 );

	personNames.clear();	// Make sure it starts as empty.
	nPersons = 0;

	// store the face images in an array
	for(iFace=0; iFace<nFaces; iFace++)
	{
		char personName[256];
		string sPersonName;
		int personNumber;
		// read person number (beginning with 1), their name and the image filename.
		fscanf(imgListFile, "%d %s %s", &personNumber, personName, imgFilename);
		sPersonName = personName;
		//ROS_INFO("Got %d: %d, <%s>, <%s>.\n", iFace, personNumber, personName, imgFilename);

		// Check if a new person is being loaded.
		if (personNumber > nPersons) {
			// Allocate memory for the extra person (or possibly multiple), using this new person's name.
			for (i=nPersons; i < personNumber; i++) {
				personNames.push_back( sPersonName );
			}
			nPersons = personNumber;
			//ROS_INFO("Got new person <%s> -> nPersons = %d [%d]\n", sPersonName.c_str(), nPersons, personNames.size());
		}

		// Keep the data
		trainPersonNumMat->data.i[iFace] = personNumber;

		// load the face image
		pfaceImg = cvLoadImage(imgFilename, CV_LOAD_IMAGE_GRAYSCALE);
                psizedImg = resizeImage(pfaceImg, faceWidth, faceHeight);
                // Give the image a standard brightness and contrast, in case it was too dark or low contrast.
                pequalizedImg = cvCreateImage(cvGetSize(psizedImg), 8, 1);	// Create an empty greyscale image
                cvEqualizeHist(psizedImg, pequalizedImg);
		faceImgArr[iFace] = pequalizedImg;
                cvReleaseImage( &pfaceImg );cvReleaseImage( &psizedImg );   
		if( !faceImgArr[iFace] )
		{
			fprintf(stderr, "Can\'t load image from %s\n", imgFilename);
			return 0;
		}
	}

	fclose(imgListFile);

	ROS_INFO("Data loaded from '%s': (%d images of %d people).\n", filename, nFaces, nPersons);
	ROS_INFO("People: ");
	if (nPersons > 0)
		ROS_INFO("<%s>", personNames[0].c_str());
	for (i=1; i<nPersons; i++) {
		ROS_INFO(", <%s>", personNames[i].c_str());
	}
	ROS_INFO(".\n");

	return nFaces;
}



// Re-train the new face rec database 
// Depending on the number of images in the training set and number of people, it might take 30 seconds or so.
bool FaceRecognitionLib::retrainOnline(void)
{
	// Free & Re-initialize the global variables.
        if(trainPersonNumMat)	{cvReleaseMat( &trainPersonNumMat );trainPersonNumMat=0;}
	int i;
	if (faceImgArr)
        {
	   for (i=0; i<nTrainFaces; i++)
              if (faceImgArr[i])	{cvReleaseImage( &faceImgArr[i] );}
	   cvFree( &faceImgArr ); // array of face images
           faceImgArr=0;	                   
        }
        if(eigenVectArr)
        {
           for (i=0; i<nEigens; i++)
              if (eigenVectArr[i])      {cvReleaseImage( &eigenVectArr[i] );}
	   cvFree( &eigenVectArr ); // eigenvectors
           eigenVectArr=0;
        }

        if(trainPersonNumMat) {cvReleaseMat( &trainPersonNumMat ); trainPersonNumMat=0;}// array of person numbers
	personNames.clear();			// array of person names (indexed by the person number). Added by Shervin.
	nPersons = 0; // the number of people in the training set. Added by Shervin.
	nTrainFaces = 0; // the number of training images
	nEigens = 0; // the number of eigenvalues
	if(pAvgTrainImg) {cvReleaseImage( &pAvgTrainImg ); pAvgTrainImg=0;}// the average image
	if(eigenValMat)  {cvReleaseMat( &eigenValMat );eigenValMat=0;} // eigenvalues
	if(projectedTrainFaceMat) {cvReleaseMat( &projectedTrainFaceMat );projectedTrainFaceMat=0;} // projected training faces
	// Retrain from the data in the files
	if(!learn("train.txt"))
           return(false);
        database_updated=true;
        return(true);
	
}
