#include <iostream>
#include <queue>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <climits>
#include <Eigen/Dense>
#include "laneDetector.hpp"

const int threshold=45;

class point{
    public: int x,y;
    point(int x_,int y_){
        x=x_;y=y_;
    }

    point(){
        x=0;y=0;
    }
};

class Point_XY_c{
public:
    double x, y, z;
    Point_XY_c() {
        x = y = z = 0;
    }
    Point_XY_c(double x_, double y_, double z_) {
        x = x_;
        y = y_;
        z = z_;
    }
    void operator=(const Point_XY_c& p_) {
        this->x = p_.x;
        this->y = p_.y;
        this->z = p_.z;
    }

    Point_XY_c operator+(const Point_XY_c& right) {
        return Point_XY_c(x + right.x, y + right.y, z + right.z);
    }

    Point_XY_c operator-(const Point_XY_c& right) {
        return Point_XY_c(x - right.x, y -right.y, z - right.z);;
    }
    Point_XY_c operator*(const double& right) {
        return Point_XY_c(x * right, y *right, z * right);
    }
    Point_XY_c operator/(const double& right) {
        return Point_XY_c(x / right, y /right, z / right);
    }
};


struct ind{
    int x; int y;
    struct ind* next;
};

struct ind *front,*rear;


cv::Mat game_binary(cv::Mat img);
void enqueue(int x,int y);
void dequeue();
void qfront(int* x,int *y);
int qempty();
cv::Mat qblobdetect(cv::Mat& bin,int** A,int* mark);
cv::Mat game_binary(cv::Mat img);
cv::Mat link_blobs(cv::Mat qblob,int** A,int mark,point* min_x,point* max_x,point* min_y,point* max_y,int* connected_index);

cv::Mat LaneDetector::SeperateLanes(cv::Mat &image){

    image=game_binary(image);

    int mark=0;

    int**A=new int*[image.rows];

    for(int i=0;i<image.rows;i++)A[i] = new int[image.cols];

    for(int i=0;i<image.rows;i++){
        for(int j=0;j<image.cols;j++)A[i][j]=-1;
    }

    cv::Mat qblob = qblobdetect(image, A, &mark);

    point* min_x = new point[mark+1];
    point* max_x = new point[mark+1];
    point* min_y = new point[mark+1];
    point* max_y = new point[mark+1];

    for(int i=0;i<=mark;i++){
        min_x[i].x=65535;
        max_x[i].x=0;
        min_y[i].y=65535;
        max_y[i].y=0;
    }

    int* connected_index = new int[mark+1];

    for(int i=0;i<=mark;i++) connected_index[i]=i;

    cv::Mat linked_image = link_blobs(qblob,A,mark,min_x,max_x,min_y,max_y,connected_index);

    return linked_image;
}

void enqueue(int x,int y){
    struct ind* n;
    n=(struct ind*)malloc(sizeof(struct ind));
    n->x=x;
    n->y=y;
    n->next=NULL;
    if(front==NULL){front=n;rear=front;}
    else {rear->next=n;rear=n;}
}

void dequeue(){
	struct ind* t;
	t=front;
	front=t->next;
	free(t);
}

void qfront(int* x,int *y){
	if(front!=NULL){
	*x=front->x;
	*y=front->y;}
	return;
}

int qempty(){
	if(front==NULL)return 1;
	return 0;}

    float dist(point p1,point p2){
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
    }


cv::Mat qblobdetect(cv::Mat& bin,int** A,int* mark){
    int i,j,x,y,wd=bin.cols,ht=bin.rows;

    std::queue<std::pair<int, int> > Q;

    for(i=0;i<ht;i++){
        for(j=0;j<wd;j++){
            if(bin.at<uchar>(i,j)==255){
                if(A[i][j]==-1){
                    *mark=*mark+1;
                    enqueue(i,j);

                    while(qempty()==0){
                        qfront(&x,&y);
                        int k,l;
                        dequeue();
                        for(k=x-1;k<=x+1;k++){
                            for(l=y-1;l<=y+1;l++){
                                if((k<ht)&&(l<wd)&&(k>=0)&&(l>=0)&&(A[k][l]==-1) && bin.at<uchar>(k,l)==255){
                                    enqueue(k,l);
                                    A[k][l]=*mark;}
                            }}
                        A[x][y]=*mark;
                    }
                    //std::cout<<*mark<<" "<<x<<","<<y<<std::endl;
                }
            }
        }}
    //cout<<"A->"<<A[10][10]<<endl;

    cv::Mat qblob(bin.rows,bin.cols,CV_8UC3);	 
    //IplImage* qblobdetect=cvCreateImage(cvGetSize(bin),IPL_DEPTH_8U,3);
    //int *a;double *c_x,*c_y;

    //a=(int*)malloc((*mark+1)*sizeof(int));
    for(i=0;i<ht;i++){
        for(j=0;j<wd;j++){
            if(A[i][j]==-1)qblob.at<cv::Vec3b>(i,j)[0]=qblob.at<cv::Vec3b>(i,j)[1]=qblob.at<cv::Vec3b>(i,j)[2]=255;
            else{	 
                qblob.at<cv::Vec3b>(i,j)[0]=255*sin(A[i][j]);
                qblob.at<cv::Vec3b>(i,j)[1]=255*cos(A[i][j]);
                qblob.at<cv::Vec3b>(i,j)[2]=255*(sin(A[i][j])+cos(A[i][j]))/1.5; 
            }
        }}

    //centroid(bin,A,*mark,count_x,count_y); 
    return qblob;
}

cv::Mat game_binary(cv::Mat img){
    cv::Mat bin(img.rows,img.cols,CV_8UC1,cvScalarAll(0));
    // TODO : img.at<cv::Vec3b>(i, j)[0] = sdfsa
    int i,j;	
    for(i=0;i<img.rows;i++){
        for(j=0;j<img.cols;j++){
            if( img.at<cv::Vec3b>(i, j)[0]>=235 || img.at<cv::Vec3b>(i, j)[1] >= 235|| img.at<cv::Vec3b>(i, j)[2] >= 235)
                bin.at<uchar>(i, j) = 255;
            else bin.at<uchar>(i, j) = 0;
        }

    }
    return bin;
}

cv::Mat link_blobs(cv::Mat qblob,int** A,int mark,point* min_x,point* max_x,point* min_y,point* max_y,int* connected_index){
    for(int i=0;i<qblob.rows;i++){
        for(int j=0;j<qblob.cols;j++){ int r=A[i][j];
            if(r!=-1){
                if(i>max_x[r].x){max_x[r].x=i;max_x[r].y=j;}
                if(i<min_x[r].x){min_x[r].x=i;min_x[r].y=j;}
                if(j>max_y[r].y){max_y[r].x=i;max_y[r].y=j;}
                if(j<min_y[r].y){min_y[r].x=i;min_y[r].y=j;}
            }
        }
    }
    for(int i=1;i<=mark;i++){
        for(int j=i+1;j<=mark;j++){
            if(dist(max_x[i],min_x[j])<threshold || dist(min_x[i],max_x[j])<threshold || dist(min_y[i],max_y[j])<threshold ||dist(max_y[i],min_y[j])<threshold){
                connected_index[j]=connected_index[i];
                std::cout<<" if condition running"<<i<<" "<<j<<std::endl;
            }
            else if(dist(max_x[i],max_x[j])<threshold || dist(min_x[i],min_x[j])<threshold || dist(min_y[i],min_y[j])<threshold ||dist(max_y[i],max_y[j])<threshold){
                connected_index[j]=connected_index[i];
                std::cout<<"else if condition running"<<i<<" "<<j<<std::endl;
            }	
        }			
    }
    for(int i=0;i<=mark;i++)std::cout<<connected_index[i]<<" "<<min_x[i].x<<" "<<max_x[i].x<<" "<<min_y[i].y<<" "<<max_y[i].y<<std::endl;
    cv::Mat linked(qblob.rows,qblob.cols,CV_8UC3);
    for(int i=0;i<qblob.rows;i++){
        for(int j=0;j<qblob.cols;j++){
            if(A[i][j]==-1)qblob.at<cv::Vec3b>(i,j)[0]=qblob.at<cv::Vec3b>(i,j)[1]=qblob.at<cv::Vec3b>(i,j)[2]=255;
            else{	 
                linked.at<cv::Vec3b>(i,j)[0]=255*sin(connected_index[A[i][j]]);
                linked.at<cv::Vec3b>(i,j)[1]=255*cos(connected_index[A[i][j]]);
                linked.at<cv::Vec3b>(i,j)[2]=255*(sin(connected_index[A[i][j]])+cos(connected_index[A[i][j]]))/1.5; 
            }
        }
    }
    return linked;

}
