#include <iostream>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <Eigen/Dense>
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

class Curve_C{
public:
	Curve_C(){
	}	
	std::vector<double> constants;
	int degree;
	void make_curve(std::vector<point> &p, int degree_){
		degree = degree_;
		Eigen::MatrixXd m(p.size(),degree+1);
		Eigen::MatrixXd y(p.size(),1);
		for(int r=0;r<p.size();r++){
			for(int c=0;c<=degree;c++){
				m(r,c)=pow(p[r].x,c);
				y(r,0)=p[r].y;
			}
		}
		Eigen::MatrixXd c(degree+1,1);
		c = ((m.transpose()*m).inverse())*(m.transpose()*y);
		constants.clear();
		for(int i=0;i<=degree;i++){
			constants.push_back(c(i,0));
			std::cout<<c(i,0)<<"\t";
		}
		std::cout<<std::endl;
	}	
};


struct ind{int x;int y;
struct ind* next;};
struct ind *front,*rear;


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

struct poly{
	int degree;
	float* coeff;
};

struct poly add(struct poly p1,struct poly p2){
	int n=p1.degree,m=p2.degree;
int ld=n>m?m:n;
struct poly p3;
p3.degree=n+m-ld;
p3.coeff=new float[p3.degree+1];
for(int i=0;i<=ld;i++)p3.coeff[i]=p1.coeff[i]+p2.coeff[i];
for(int i=ld+1;i<=p3.degree;i++){
	if(ld==n)p3.coeff[i]=p2.coeff[i];
	else p3.coeff[i]=p1.coeff[i];}	
return p3;}

struct poly mult(struct poly p1,struct poly p2){
	struct poly p3;
	p3.degree=p1.degree+p2.degree;
	p3.coeff=new float[p3.degree+1];
	for(int i=0;i<=p3.degree;i++)p3.coeff[i]=0;
for(int i=0;i<=p1.degree;i++){
	for(int j=0;j<=p2.degree;j++)p3.coeff[i+j]+=p1.coeff[i]*p2.coeff[j];
}
return p3;
}

struct poly langrange(point* P){
	struct poly p1,p2;
	p1.degree=p2.degree=0;
	p1.coeff=new float[1];
	p2.coeff=new float[1];
	p1.coeff[0]=1;
	p2.coeff[0]=0;
	for(int i=0;i<=5;i++){
		struct poly p3;
		p3.degree=1;
		p3.coeff=new float[2];
		p3.coeff[0]=1;
		float denom=1;
		for(int j=0;j<=5;j++){
			if(j!=i){denom*=(P[i].x-P[j].x);
				p3.coeff[1]=-P[j].x;
				p1=mult(p1,p3);}
		}
		denom/=P[i].y;
		struct poly pt;
		pt.degree=0;
		pt.coeff=new float[1];
		if(denom!=0)pt.coeff[0]=1/denom;
		else pt.coeff[0]=0;
		p1=mult(p1,pt);
		p2=add(p2,p1);
		p1.degree=0;
		p1.coeff=new float[1];
		p1.coeff[0]=1;
		//std::cout<<"degree so far:"<<p2.degree<<" ";
	}
return p2;
}

cv::Mat qblobdetect(cv::Mat& bin,int** A,int* mark){
	 //std::cout<<"hello";
	 int i,j,x,y,wd=bin.cols,ht=bin.rows,max_pix_count=0,max_size_index=0;
	 //cout<<" a->"<<A[10][10]<<" "<<mark<<endl;
	 for(i=0;i<ht;i++){
		 for(j=0;j<wd;j++){
			 if(bin.at<uchar>(i,j)==255){
				 if(A[i][j]==-1){
					 *mark=*mark+1;
				 enqueue(i,j);

				 //A[i][j]=0;
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

void equation(int mark,point* min_x,point* min_y,point* max_x,point* max_y,int* connected_index,struct poly* p_lane1,struct poly* p_lane2){
int* connected_index2=new int[mark+1];
struct poly p1,p2,p3,p4;
point* max_x_lane_eq,*min_x_lane_eq,*min_y_lane_eq,*max_y_lane_eq;
	max_x_lane_eq=new point[6];
	max_y_lane_eq=new point[6];
	min_x_lane_eq=new point[6];
	min_y_lane_eq=new point[6];
for(int i=0;i<=mark;i++)connected_index2[i]=connected_index[i];
int all_zero=0;
do{int r;
	all_zero=0;
	p1.degree=p2.degree=p3.degree=p4.degree=0;
p1.coeff=new float[1];
p2.coeff=new float[1];
p3.coeff=new float[1];
p4.coeff=new float[1];
p1.coeff[0]=p2.coeff[0]=p3.coeff[0]=p4.coeff[0]=0;
for(int i=0;i<=mark;i++)if(connected_index2[i]!=0){all_zero=1;r=connected_index2[i];break;}

if(all_zero){int lane_pt=0,interpolated=0;
	for(int i=0;i<=mark;i++){
		if(connected_index2[i]==r){
			lane_pt++;
			max_x_lane_eq[lane_pt%6]=max_x[i];
			max_y_lane_eq[lane_pt%6]=max_y[i];
			min_x_lane_eq[lane_pt%6]=min_x[i];
			min_y_lane_eq[lane_pt%6]=min_y[i];
if(lane_pt==6){
	for(int j=0;j<=5;j++)std::cout<<max_x_lane_eq[j].x<<" "<<max_x_lane_eq[j].y<<std::endl;
}
			if(lane_pt>=6){interpolated++;

				p1=add(p1,langrange(max_x_lane_eq));
				p2=add(p2,langrange(max_y_lane_eq));
				p3=add(p3,langrange(min_x_lane_eq));
				p4=add(p4,langrange(min_y_lane_eq));
			}
		connected_index2[i]=0;}
//std::cout<<"done interpolating till "<<i<<" "<<lane_pt<<std::endl;

}

struct poly pt;
pt.degree=0;
pt.coeff=new float[1];
pt.coeff[0]=1.0/interpolated;
std::cout<<"1/interpolated="<<pt.coeff[0]<<std::endl;
p1=mult(p1,pt);p2=mult(p2,pt);p3=mult(p3,pt);p4=mult(p4,pt);
if(r==1){*p_lane1=add(p1,add(p2,add(p3,p4)));
	
	pt.coeff[0]=0.25;
	*p_lane1=mult(*p_lane1,pt);
}
else{*p_lane2=add(p1,add(p2,add(p3,p4)));
	
	pt.coeff[0]=0.25;
	*p_lane2=mult(*p_lane2,pt);
}
}
}while(all_zero!=0);
free(connected_index2);
free(max_x_lane_eq);
free(max_y_lane_eq);
free(min_x_lane_eq);
free(min_y_lane_eq);
} 



int main(){
    cv::Mat road=cv::imread("road_isolated.jpg",CV_LOAD_IMAGE_COLOR);
    cv::namedWindow("road",CV_WINDOW_AUTOSIZE);
    road=game_binary(road);
    cv::imshow("road",road);
    cv::waitKey(0);

    int mark=0;
    int**A=new int*[road.rows];
    for(int i=0;i<road.rows;i++)A[i]=new int[road.cols];
    for(int i=0;i<road.rows;i++){
        for(int j=0;j<road.cols;j++)A[i][j]=-1;
    }
    cv::Mat qblob=qblobdetect(road,A,&mark);
    cv::imshow("road",qblob);
    cv::waitKey(0);
    int start_s=clock();
    point* min_x=new point[mark+1];
    point* max_x=new point[mark+1];
    point* min_y=new point[mark+1];
    point* max_y=new point[mark+1];
    for(int i=0;i<=mark;i++){
        min_x[i].x=65535;
        max_x[i].x=0;
        min_y[i].y=65535;
        max_y[i].y=0;
    }
    int* connected_index=new int[mark+1];
    for(int i=0;i<=mark;i++)connected_index[i]=i;
    cv::Mat linked_road=link_blobs(qblob,A,mark,min_x,max_x,min_y,max_y,connected_index);
    cv::imshow("road",linked_road);
    cv::waitKey(0);
    std::vector<point> max_x_v,min_x_v,max_y_v,min_y_v;
    for(int i=1;i<=mark;i++){
        max_x_v.push_back(max_x[i]);
        min_x_v.push_back(min_x[i]);
        min_y_v.push_back(min_y[i]);
        max_y_v.push_back(max_y[i]);
    }
    /*struct poly p1,p2;
      p1.degree=p2.degree=1;
      p1.coeff=new float[2];
      p2.coeff=new float[2];
      p1.coeff[0]=p1.coeff[1]=p2.coeff[0]=p2.coeff[1]=1;
      struct poly p3=mult(p1,p2);
      for(int i=0;i<=2;i++)std::cout<<p3.coeff[i]<<" ";*/

    Curve_C cu;
    cu.make_curve(max_x_v,5);

    point* P=new point[6];
    for(int i=0;i<=5;i++){
        P[i].x=i;
        P[i].y=i*i+2*i-1;
    }
    for(int i=0;i<=cu.degree;i++)std::cout<<cu.constants.at(i)<<" ";
    std::cout<<std::endl;
    cv::Mat func(road.rows,road.cols,CV_8UC3);
    for(int i=0;i<road.rows;i++){
        int j=cu.constants.at(0)+i*(cu.constants.at(1)+i*(cu.constants.at(2)+i*(cu.constants.at(3)+i*(cu.constants.at(4)+i*cu.constants.at(5)))))/1;
        //int k=p_lane2.coeff[5]+i*(p_lane2.coeff[4]+i*(p_lane2.coeff[3]+i*(p_lane2.coeff[2]+i*(p_lane2.coeff[1]+i*p_lane2.coeff[0]))))/10;
        //int l=p1.coeff[5]+i*(p1.coeff[4]+i*(p1.coeff[3]+i*(p1.coeff[2]+i*(p1.coeff[1]+i*p1.coeff[0]))));;
        if(j>=0 && j<road.cols)cv::circle(func,cv::Point(i,j),4,cv::Scalar(255,0,0),-1);
        //if(k>=0 && k<road.cols)cv::circle(func,cv::Point(i,k),4,cv::Scalar(0,255,0),-1);
        //if(l>=0 && l<road.cols)cv::circle(func,cv::Point(i,l),4,cv::Scalar(0,0,255),-1);
    }
    cv::namedWindow("function",CV_WINDOW_AUTOSIZE);
    cv::imshow("function",func);
    int stop_s=clock();
    cv::waitKey(0);
    /*struct poly p1=langrange(P);
      std::cout<<p1.degree<<std::endl;
      for(int i=0;i<=p1.degree;i++)std::cout<<p1.coeff[i]<<" ";
      std::cout<<std::endl;
      struct poly p_lane1,p_lane2;
      p_lane1.degree=p_lane2.degree=0;
      p_lane1.coeff=new float[1];
      p_lane2.coeff=new float[1];
      p_lane1.coeff[0]=p_lane2.coeff[0]=0;
      equation(mark,min_x,min_y,max_x,max_y,connected_index,&p_lane1,&p_lane2);

      for(int i=0;i<=p_lane1.degree;i++)std::cout<<p_lane1.coeff[i]<<" ";std::cout<<std::endl;
      for(int i=0;i<=p_lane2.degree;i++)std::cout<<p_lane2.coeff[i]<<" ";std::cout<<std::endl;

      cv::Mat func(road.rows,road.cols,CV_8UC3);
      for(int i=0;i<road.rows;i++){
      int j=p_lane1.coeff[5]+i*(p_lane1.coeff[4]+i*(p_lane1.coeff[3]+i*(p_lane1.coeff[2]+i*(p_lane1.coeff[1]+i*p_lane1.coeff[0]))))/1;
      int k=p_lane2.coeff[5]+i*(p_lane2.coeff[4]+i*(p_lane2.coeff[3]+i*(p_lane2.coeff[2]+i*(p_lane2.coeff[1]+i*p_lane2.coeff[0]))))/10;
    //int l=p1.coeff[5]+i*(p1.coeff[4]+i*(p1.coeff[3]+i*(p1.coeff[2]+i*(p1.coeff[1]+i*p1.coeff[0]))));;
    if(j>=0 && j<road.cols)cv::circle(func,cv::Point(i,j),4,cv::Scalar(255,0,0),-1);
    if(k>=0 && k<road.cols)cv::circle(func,cv::Point(i,k),4,cv::Scalar(0,255,0),-1);
    //if(l>=0 && l<road.cols)cv::circle(func,cv::Point(i,l),4,cv::Scalar(0,0,255),-1);
    }
    cv::namedWindow("function",CV_WINDOW_AUTOSIZE);
    cv::imshow("function",func);

    cv::waitKey(0);*/
    std::cout<<"time:"<< (stop_s-start_s)/double(CLOCKS_PER_SEC) <<std::endl;
}
