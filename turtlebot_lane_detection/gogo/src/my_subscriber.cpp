#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#define STORE_LINE 4

using namespace std;
using namespace cv;


vector<Vec4i> meanLineVec;
vector<Vec4i> tempLineVec;

vector<Vec4i>::iterator it;

vector<Vec4i>::iterator itL;
vector<Vec4i>::iterator itR;

vector<Vec4i> stopVec;
vector<Vec4i> myLRecentBest;
vector<Vec4i> leftVec;
vector<Vec4i> rightVec;
vector<Vec4i> lines;
vector<int> meanWidth;

vector<Mat> channels;
Size MyrgbSize;

sensor_msgs::Image im_temp;
int nBlue=0, nGreen=0, nRed=0;
int tmpBool = 0;
Mat grayImg, blurImg, cannyImg, edgeImg, myROI, colorROI, myred, myred_b, mylabel, stats, centroids;
Mat frame;
float colsOffset, rowsOffset;

int count = 0;

float ladian;
int degree;
int numOfLables;
int max_ = -1, idx_ = 0;
int left_, top_, width_, height_;
vector<Vec4i> hierarchy;
 Mat numbox,numbox2;
vector<Rect> boundRect;
vector<vector<Point> > contours_poly;
vector<Rect> mybound;
vector<vector<Point> > mycontours;

int mycontourSize;
int delay;
int tmp1,tmp2;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  try
  {   //CvShare에는 Image가 포인팅하는 형태로 넣을수 x..
     im_temp = *msg;
     //cv::imshow("view",cv_bridge::toCvShare(msg,"rgb8")->image);
    //cv_bridge::CvImagePtr myImg = cv_bridge::toCvCopy(im_temp, "rgb8");
    //VideoCapture inputVideo(1);
    
    frame = cv_bridge::toCvCopy(im_temp, "rgb8")->image;
if(!(frame.empty()))
{	
    myROI = frame(
      Rect(frame.cols*0.1, 
           frame.rows*0.2, 
           frame.cols - (frame.cols*0.06) - frame.cols*0.1, 
           frame.rows - (frame.rows*0.07) - frame.rows*0.2)); 

     // std::cout  << "frame ERROR!!!!!!"<< frame.rows << std::endl;
   colorROI = frame(
     Rect(frame.cols*0.1, 
          frame.rows*0.2, 
          frame.cols - (frame.cols*0.06) - frame.cols*0.1, 
          frame.rows - (frame.rows*0.07) - frame.rows*0.2));
    
      colsOffset = frame.cols*0.1;
      rowsOffset = frame.rows*0.2;
     

      cvtColor(myROI, grayImg, COLOR_BGR2GRAY);
      GaussianBlur(grayImg, blurImg, Size(5, 5), 0.5);
     
      Canny(blurImg, cannyImg, 80, 15, 3);
      HoughLinesP(cannyImg, lines, 1, CV_PI / 180.0, 80, 80, 5);
      //이 얻어낸 좌우의 라인을 정사각형의 roi에 넣어서 각도를 구분하자.
      //정사각형의 윈도우를 이동시켜서 좌,우 선을 구분.
      cvtColor(colorROI, myred, COLOR_BGR2HSV);
      inRange(myred, Scalar(0, 50,170), Scalar(255, 255, 180), myred_b);

      numOfLables = connectedComponentsWithStats(myred_b, mylabel, stats, centroids, 8, CV_32S);
      
      for (int j = 1; j < numOfLables; j++) {
         int area = stats.at<int>(j, CC_STAT_AREA);
         if (max_ < area) {
            max_ = area;
            idx_ = j;
         }
      }



      left_ = stats.at<int>(idx_, CC_STAT_LEFT);
      top_ = stats.at<int>(idx_, CC_STAT_TOP);
      width_ = stats.at<int>(idx_, CC_STAT_WIDTH);
      height_ = stats.at<int>(idx_, CC_STAT_HEIGHT);
      cout << left_ << "," << top_ << "," << width_ << "," << height_ << endl;

      rectangle(colorROI, 
                Point(left_, top_), 
                Point(left_ + width_, top_ + height_), 
                Scalar(0, 255, 255), 
                3); 
       

      //대응하는 윤곽선이 없으면 음수를 가진다.
     
      numbox = cannyImg(Rect(
                             Point(left_,top_),
                             Point(left_+width_, top_+height_)
                             )
                        );
      numbox2 = colorROI(Rect(
                             Point(left_, top_), 
                             Point(left_ + width_, top_ + height_)
                             )
                        );
 

     findContours(numbox, mycontours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point());
     vector<Rect> boundRect(mycontours.size()); //윤곽선배열벡터 크기에 맞는 사각형 배열벡터 생성
     vector<vector<Point> > contours_poly(mycontours.size());
 


for (int i = 0; i < mycontours.size(); i++)
      {
         approxPolyDP(Mat(mycontours[i]), contours_poly[i], 1, true);//다각형을 명시된 도형으로 전환. 세번째 인자는 원래 곡선과 근사값의 차이를 지정하고 네번째 인자는 폐곡선 여부
         boundRect[i] = boundingRect(Mat(contours_poly[i]));
      }
      
      Rect calboundRect(Point(left_, top_), Point(left_ + width_, top_ + height_));
      vector<Rect> mybound = boundRect;
      Rect tmp;
//   if(contours.size()>0)

      /*      for (int i = 0; i < contours.size(); i++)
      {
         for (int j = 1; j < contours.size() - i-1; j++)
         {
            if (boundRect[j].height < boundRect[j - 1].height)
               tmp = boundRect[j-1];
               boundRect[j-1] = boundRect[j];
               boundRect[j] = tmp;
         }
      }
   */   
   if (mycontours.size()>0)
      for (int i=0, j=i+1; i < mycontours.size()-1; i++)
         if (boundRect[i].width == 0) boundRect[i].width = 1;

         if(boundRect[i].width<(calboundRect.width)/2)
         //   if (abs(degree)>75 && abs(degree)<95)
//               if(boundRect[i].height/boundRect[i].width>3)
                  rectangle(numbox2, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 255), 1, 8, 0);   
      }

////////////////////for slide window////////////////////////////////////////////////////////


Mat slideWindow = frame(
     Rect(frame.cols*0.1, 
          frame.rows*0.2, 
          frame.cols - (frame.cols*0.06) - frame.cols*0.1, 
          frame.rows - (frame.rows*0.07) - frame.rows*0.2));//10cm x 10cm roi(calibration ok)
for(int i=0; i<slideWindow.x; i++)
{
  for(int j=0; j<slideWindow.y; j++)
    {
      
    }
}




/////////////////////////////////////////////////////////////////////////////
    if(!lines.empty())
{


       it = lines.end() - 1;


         ladian = atan2f((*it)[3] - (*it)[1], (*it)[2] - (*it)[0]);
         degree = ladian * 180 / CV_PI;
        if (abs(degree) > 25 && abs(degree) < 36)
         {
            if (degree <= 0)
            {
               leftVec.push_back(*it);
               cout << "left : " << leftVec.back();
               line(frame, Point((*it)[0] + colsOffset, (*it)[1] + rowsOffset), Point((*it)[2] + colsOffset, (*it)[3] + rowsOffset), Scalar(255,0, 0 ), 3, 0);

            }
            else
            {
               rightVec.push_back(*it);
               cout << "right : " << rightVec.back();
               line(frame, Point((*it)[0] + colsOffset, (*it)[1] + rowsOffset), Point((*it)[2] + colsOffset, (*it)[3] + rowsOffset), Scalar(255, 0, 0), 3, 0);
            }
            //if (!leftVec.empty() && !rightVec.empty())//(leftVec.capacity() > 4 && rightVec.capacity() > 4)//capacity()의 경우 pushback의 반복때마다 내 예상보다 더 늘어날 수 있어(원소는 없는데 공간만 느는..) 따라서 empty로 체크하자.
         //      meanWidth.push_back(abs((*itL)[0] - (*itR)[0]));

         }

else
            if (abs(degree) >= 180 && abs(degree) < 190)
               stopVec.push_back(*it);//정지선벡터에 정지선 좌표 정보 저장

         if (!meanWidth.empty())
            cout << "mean : " << meanWidth.back() << endl;
         if (!leftVec.empty())
         {
            itL = leftVec.end() - 1;
            //line(frame, Point((*itL)[0] + colsOffset, (*itL)[1] + rowsOffset), Point((*itL)[2] + colsOffset, (*itL)[3] + rowsOffset), Scalar(0, 0, 255), 3, 0);
         }
         if (!rightVec.empty())
         {
            itR = rightVec.end() - 1;
            //line(frame, Point((*itR)[0] + colsOffset, (*itR)[1] + rowsOffset), Point((*itR)[2] + colsOffset, (*itR)[3] + rowsOffset), Scalar(0, 0, 255), 3, 0);
         }
         if (!stopVec.empty())
         {
            //   line(frame, Point(stopVec[0][0] + colsOffset, stopVec[0][1] + rowsOffset), Point(stopVec[0][2] + colsOffset, stopVec[0][3] + rowsOffset), Scalar(0, 0, 255), 3, 0);
         }





}
MyrgbSize = myROI.size();
for (int i = 0; i < MyrgbSize.height; i++) {//image 의 bgr 배열 변경
		for (int j = 0; j < MyrgbSize.width; j++) {
			nBlue = myROI.at<Vec3b>(i, j)[0];
			nGreen = myROI.at<Vec3b>(i, j)[1];
			nRed = myROI.at<Vec3b>(i, j)[2];
			//픽셀의 채널 위치들을 옆으로 한칸씩 이동
			tmp1 = myROI.at<Vec3b>(i, j)[0];
			tmp2 = myROI.at<Vec3b>(i, j)[1];
			myROI.at<Vec3b>(i, j)[0] = myROI.at<Vec3b>(i, j)[1];//B값이 저장된 픽셀에 G를
			myROI.at<Vec3b>(i, j)[1] = tmp2;//G값이 저장된 픽셀에 R을
			myROI.at<Vec3b>(i, j)[2] = tmp1;//R값이 저장된 픽셀에 B를 
			//cout << "Red : " << nRed << " , Green : " << nGreen << " , Blue : " << nBlue << endl;  // image 전체의 rgb 값을 출력
		}
	}


     imshow("myLine", myROI);
     imshow("myred_b", myred_b);
     imshow("colorROI", colorROI);

    // imshow("view", frame);  
  //  imshow("s",myROI);
    int ckey = waitKey(10);
    if(ckey == 27) exit(1);
  }


}




  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  ros::Rate loop_rate(5);
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1,    imageCallback);
  while (nh.ok()) {
    // cv_bridge::CvImagePtr myImg = cv_bridge::toCvCopy(im_temp, "rgb8");
  //   cv::Mat i = myImg->image;
  //   cv::Mat myImg = im_temp;
   // cv::Mat img_scaled_8u;
    //cv::Mat(im_temp->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (1000 - 0));
    //cv::cvtColor(img_scaled_8u, myImg, CV_GRAY2RGB);
//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//cv::circle(dImg, cv::Point(px, py), 10, CV_RGB(255,0,0));
// Update GUI Window
  //cv::imshow("view", cv_bridge::toCvCopy(im_temp, "rgb8")->image);
    //cv::imshow("WINDOW",*k);
//    cv::waitKey(3);
    ros::spinOnce();
    loop_rate.sleep();
  }

  //cv::destroyWindow("view");
}
