#include "ros/ros.h"                                            // ROS 기본 헤더파일 
#include "gogo/MsgTutorial.h"        // msgTutorial 메시지 파일 헤더 (빌드후 자동 생성됨)

int main(int argc, char **argv)                                 // 노드 메인 함수
{
        ros::init(argc, argv, "topic_publisher");  // 노드명 초기화
        ros::NodeHandle nh;                                     // ROS 시스템과 통신을 위한 노드 핸들 선언

        // 퍼블리셔 선언, pubnsub 패키지의 msgTutorial 메시지 파일을 이용한
        // 퍼블리셔 ros_tutorial_pub 를 작성한다. 토픽명은 ＂ros_tutorial_msg＂ 이며,
        // 퍼블리셔의 큐(queue) 사이즈를 100개로 설정한다는 것이다
        ros::Publisher ros_tutorial_pub = nh.advertise<gogo::MsgTutorial>("ros_tutorial_msg", 100);
        ros::Rate loop_rate(10);        // 루프 주기를 10Hz로 설정
        int count = 0;                  // 메시지에 사용될 변수 선언

        while (ros::ok())
        {

		
                gogo::MsgTutorial msg;      // msgTutorial 메시지 파일 형식으로 msg 라는 메시지를 선언
                
		msg.stamp = ros::Time::now();
		msg.data = count;                       // count 라는 변수를 이용하여 메시지 값을 정한다.
               
		ROS_INFO("send msg = %d", count);   // ROS 함수를 이용하여 count 변수를 표시한다.
		ROS_INFO("send msg = %d", msg.stamp.sec);		
		ROS_INFO("send msg = %d", msg.stamp.nsec);
		ROS_INFO("send msg = %d", msg.data);
                ros_tutorial_pub.publish(msg);          // 메시지를 발행한다. 약 0.1초 간격으로 발행된다.
                loop_rate.sleep();                              // 위에서 정한 루프 주기에 따라 슬립에 들어간다.
                ++count;                                        // count 변수 1씩 증가
        }
        return 0;
}

