#include <iostream>
#include <cmath>
#include "/usr/include/opencv/cv.h"
#include "/usr/include/opencv/highgui.h"
#include "/usr/include/opencv2/opencv.hpp"
#include "/usr/include/opencv2/core/core.hpp"
#include "/usr/include/opencv2/highgui/highgui.hpp"
#include "/usr/include/opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#define PI 3.14159265359

IplImage* imgTracking=0;
CvFont font;
int fontface = CV_FONT_HERSHEY_SIMPLEX;

int thickness = 2;
double a,b,c,A,B,C,cosA,cosB,cosC;


void angle ( Point pt0, Point pt1, Point pt2){
		
		 a = sqrt(  ((pt1.x-pt0.x)*(pt1.x-pt0.x)) + ((pt1.y-pt0.y)*(pt1.y-pt0.y))   );
		 b = sqrt(  ((pt2.x-pt0.x)*(pt2.x-pt0.x)) + ((pt2.y-pt0.y)*(pt2.y-pt0.y))   );
		 c = sqrt(  ((pt1.x-pt2.x)*(pt1.x-pt2.x)) + ((pt1.y-pt2.y)*(pt1.y-pt2.y))   );
		 A = (acos( ((b*b) + (c*c) - (a*a))/2*b*c))*(180/PI);
		 B = (acos( ((a*a) + (c*c) - (b*b))/2*a*c))*(180/PI);
		 C = (acos( ((a*a) + (b*b) - (c*c))/2*a*b))*(180/PI);

	
		
}

void trackObject(IplImage* imgThresh){
	CvSeq* contour;  //hold the pointer to a contour
	CvSeq* result;     //hold sequence of points of a contour
    CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours
 
        //finding all contours in the image
    cvFindContours(imgThresh, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
  
       //iterating through each contour
    while(contour)
    {
        //obtain a sequence of points of the countour, pointed by the variable 'countour'
       result = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contour)*0.02, 0);
          
       //if there are 3 vertices  in the contour and the area of the triangle is more than 100 pixels
       if(result->total==3 && fabs(cvContourArea(result, CV_WHOLE_SEQ))>100 )
       {
       //iterating through each point
          CvPoint *pt[3];
          for(int i=0;i<3;i++){
			pt[i] = (CvPoint*)cvGetSeqElem(result, i);
          }
         
		 a = sqrt(  (((pt[1]->x)-(pt[0]->x))^2) + (((pt[1]->y)-(pt[0]->y))^2)   );  // Find The Angle
		 b = sqrt(  (((pt[2]->x)-(pt[0]->x))^2) + (((pt[2]->y)-(pt[0]->y))^2)   );
		 c = sqrt(  (((pt[2]->x)-(pt[1]->x))^2) + (((pt[2]->y)-(pt[1]->y))^2)   );
		 cosA = ((b*b) + (c*c) - (a*a))/(2*b*c);
		 cosB = ((a*a) + (c*c) - (b*b))/(2*a*c);
		 cosC = ((a*a) + (b*b) - (c*c))/(2*a*b);
		 A = acos(cosA)* 180/PI;
		 B = acos(cosB)* 180/PI;
		 C = acos(cosC)* 180/PI; 
         
        cvInitFont(&font, fontface, 0.5 , 0.5 , 0 , thickness , 8);
         
         if (  A == 90 || B == 90 || C == 90  ){
			// cvPutText(imgTracking, "Segitiga Siku2", *pt[0], &font , CV_RGB(255,255,255));
			cout << "Segitiga Siku-Siku" << endl;
			
			
			cout << "A = " << A << endl;
			cout << "B = " << B << endl;
			cout << "C = " << C << endl;
		}
		else{
			// cvPutText(imgTracking, "Segitiga", *pt[0], &font , CV_RGB(255,255,255));
			cout << "Segitiga" << endl;
			
			
			cout << "A = " << A << endl;
			cout << "B = " << B << endl;
			cout << "C = " << C << endl;
		}
         
         
		}
  
            //obtain the next contour
            contour = contour->h_next; 
      }

      cvReleaseMemStorage(&storage);
}

int main(){
    //load the video file to the memory
    CvCapture *capture =  cvCaptureFromFile("movie.mp4");

    if(!capture){
        printf("Capture failure\n");
        return -1;
    }
      
    IplImage* frame=0;
    frame = cvQueryFrame(capture);           
    if(!frame) return -1;
   
    //create a blank image and assigned to 'imgTracking' which has the same size of original video
    imgTracking=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 3);
    cvZero(imgTracking); //covert the image, 'imgTracking' to black

    cvNamedWindow("Video");     

    //iterate through each frames of the video     
    while(true){

        frame = cvQueryFrame(capture);           
        if(!frame) break;
        frame=cvCloneImage(frame); 
         
        //smooth the original image using Gaussian kernel
        cvSmooth(frame, frame, CV_GAUSSIAN,3,3); 

		//converting the original image into grayscale
		IplImage* imgGrayScale = cvCreateImage(cvGetSize(frame), 8, 1); 
		cvCvtColor(frame,imgGrayScale,CV_BGR2GRAY);
          
		//thresholding the grayscale image to get better results
		cvThreshold(imgGrayScale,imgGrayScale,100,255,CV_THRESH_BINARY_INV);
            
        //track the possition of the ball
        trackObject(imgGrayScale);

        // Add the tracking image and the frame
        cvAdd(frame, imgTracking, frame);
             
        cvShowImage("Video", frame);
   
        //Clean up used images
        cvReleaseImage(&imgGrayScale);            
        cvReleaseImage(&frame);

        //Wait 10mS
        int c = cvWaitKey(10);
        //If 'ESC' is pressed, break the loop
        if((char)c==27 ) break;      
    }

    cvDestroyAllWindows();
    cvReleaseImage(&imgTracking);
    cvReleaseCapture(&capture);     

    return 0;
}




