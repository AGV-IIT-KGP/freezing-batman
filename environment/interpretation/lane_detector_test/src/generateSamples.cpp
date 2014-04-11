#include <image_transport/image_transport.h>

#include <ros/ros.h>

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

char NORMAL[]= "\033[0m";
char RED[]= "\033[0;31m";
char BLUE[]= "\033[0;34m";
char WHITE[]= "\033[37;01m";

std::ofstream commonFile;
std::ofstream commonFile2;

cv::Mat img,temp;

std::string name;

int size_sample=8;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	int countP,countN;
	int countP2,countN2;
	cv::Mat imgRoi;
     if  ( event == cv::EVENT_LBUTTONDOWN ){
          std::cout << BLUE << "Saving positive sample - position (" << x << ", " << y << ")" << NORMAL << std::endl;
          imgRoi = temp(cv::Rect(x-size_sample/2, y-size_sample/2, size_sample, size_sample));
          
          commonFile<<"+1 ";
          commonFile2<<"+1 ";
          countP=0;
          countP2=1;
          for(int i=0;i<size_sample;i++){
			  for(int j=0;j<size_sample;j++){
				  commonFile<<countP++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[0]<<" ";
				  commonFile<<countP++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[1]<<" ";
				  commonFile<<countP++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[2]<<" ";
				  
				  commonFile2<<countP2++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[0]<<" ";
				  commonFile2<<countP2++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[1]<<" ";
				  commonFile2<<countP2++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[2]<<" ";
			  }
		  }
          
          commonFile<<countP++<<":-1"<<std::endl;
          commonFile2<<std::endl;
          cv::rectangle( img, cv::Point( x-size_sample/2, y-size_sample/2 ), cv::Point( x+size_sample/2, y+size_sample/2 ), cv::Scalar( 255, 255, 255 ));
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN ){
		 std::cout << RED << "Saving negative sample - position (" << x << ", " << y << ")" << NORMAL << std::endl;
		 imgRoi = temp(cv::Rect(x-size_sample/2, y-size_sample/2, size_sample, size_sample));
		 
		 commonFile<<"-1 ";
		 commonFile2<<"-1 ";
		 countN=0;
		 countN2=1;
		 
          for(int i=0;i<size_sample;i++){
			  for(int j=0;j<size_sample;j++){
				commonFile<<countN++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[0]<<" ";
				commonFile<<countN++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[1]<<" ";
				commonFile<<countN++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[2]<<" ";
				
				commonFile2<<countN2++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[0]<<" ";
				commonFile2<<countN2++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[1]<<" ";
				commonFile2<<countN2++<<":"<<(int)imgRoi.at<cv::Vec3b>(i,j)[2]<<" ";
			  }
		  }
          
          commonFile<<countN++<<":-1"<<std::endl;
          commonFile2<<std::endl;
          cv::rectangle( img, cv::Point( x-size_sample/2, y-size_sample/2 ), cv::Point( x+size_sample/2, y+size_sample/2 ), cv::Scalar( 0, 0, 0 ));
     }
}


int main(int argc, char** argv) {
	if(argc<2){
		printf("Usage: <name> <image_file> <optional:= kernel_size>\n");
		return -1;
	}
	commonFile.open("Samples",std::fstream::app);
	commonFile2.open("Samples2",std::fstream::app);
	img=cv::imread(argv[1]);
	
	if( argc == 3 ){
		int temp_size_sample = atoi(argv[2]);
		if( temp_size_sample>=5 && temp_size_sample<=30){
			size_sample = temp_size_sample;
		}
	}
	printf("Sample Size: %d\n",size_sample);

	temp=img.clone();
	if ( img.empty() ){ 
          std::cout << "Error loading the image" << std::endl;
          return -2; 
    }
	cv::namedWindow("Original Image");
	cv::setMouseCallback("Original Image", CallBackFunc, NULL);
	while(true){
		cv::imshow("Original Image",img);
		cv::waitKey(10);
	}
    return 0;
}
