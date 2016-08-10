#include <iostream>
#include <opencv.hpp>
#include <cstring>
#include <Kinect.h>
#include <windows.h>
#include "Serial.h"
#include "Serial.cpp"
#define NORMAL 9
#define ZERO 1
#define CENTER 2
#define ROUND 3
#define STOP 4
#define SLEEP 5
#define YES	 1
#define NO 0
using namespace cv;
using namespace std;
const string  get_name(int n);    //此函数判断出关节点的名字
bool send(int ctl, int x, int y);
void reflashdata();
void tructbar();
void iniKinect();
int height = 0, width = 0;
int speed = 40000,yuzhi=3;
int is_tracking = NO;
IKinectSensor   * mySensor = nullptr;

IBodyFrameSource    * myBodySource = nullptr;
IBodyFrameReader    * myBodyReader = nullptr;

IDepthFrameSource   * myDepthSource = nullptr;
IDepthFrameReader   * myDepthReader = nullptr;

IFrameDescription   * myDescription = nullptr;

IBodyFrame  * myBodyFrame = nullptr;
IDepthFrame * myDepthFrame = nullptr;

Mat img16, img8;
HandState leftHandState, rightHandState;
////////////////////////////////////
typedef struct tructvar { int min, max; }tructvar;
tructvar bodydepth = { 200,250 }, deta = {500,5000};
IBody   * bodyArr[BODY_COUNT];
Point3d lefthand[2], righthand[2], nohand = { 100,100,100 };
Point arduino;
CSerial serial;
int tracked;
///////////////////////////////////////
int main()
{
	lefthand[0] = lefthand[1] = nohand;
	if (serial.OpenSerialPort(_T("COM4:"), 9600, 8, 1) == false) return -1;
	iniKinect();
	tructbar();

	while (1)
	{

		tracked = -1;
		reflashdata();
		while (is_tracking==NO)
		{
			reflashdata();
			cout << "waiting~~~" << endl;
			send(SLEEP, 0, 0);
			waitKey(1);
		}
		cout << "ok"<<endl;
		send(CENTER, 0, 0);
		arduino = Point(18000/2, 27000/2);
		reflashdata();

		while (is_tracking)
		{
			waitKey(1);
			int detax=0, detay=0;
			detax+= (lefthand[0].x - lefthand[1].x)*speed;
			detay+= (lefthand[0].y - lefthand[1].y)*speed;
			detax+= (righthand[0].x - righthand[1].x)*speed;
			detay+= (righthand[0].y - righthand[1].y)*speed;
			if (abs(detax) >= deta.max || abs(detax) <=deta.min) detax = 0;
			if (abs(detay) >= deta.max || abs(detay) <= deta.min) detay = 0;
			arduino.x += detax;//减少抖动
			arduino.y += detay;
			if (arduino.x < 0) arduino.x = 0;
			if (arduino.y < 0) arduino.y = 0;
			if (arduino.x >18000) arduino.x = 18000;
			if (arduino.y >27000) arduino.y = 27000;
			cout << arduino;
			if ((righthand[0].z - righthand[1].z < -0.05 && rightHandState!=HandState_Open)|| (lefthand[0].z - lefthand[1].z < -0.05 && leftHandState != HandState_Open))
				send(ROUND, arduino.x, arduino.y),cout<<"     ROUND"<<endl;
			else
				send(NORMAL, arduino.x, arduino.y);
			reflashdata();//重写IS_HUMAN
			if (waitKey(30) == VK_ESCAPE)
				break;
		}

		if (waitKey(30) == VK_ESCAPE)
			break;
		//Sleep(100);    //为避免数据刷太快，每秒钟更新一次

	}
	myBodyReader->Release();
	myDepthReader->Release();
	myBodySource->Release();
	myDepthSource->Release();
	mySensor->Close();
	mySensor->Release(); 
	if (myDepthFrame != nullptr)myDepthFrame->Release();

	return  0;
}

const   string  get_name(int n)
{
	switch (n)
	{
	case    2:return    "Neck"; break;
	case    3:return    "Head"; break;
	case    4:return    "Left shoulder"; break;
	case    8:return    "Right shoulder"; break;
	case    7:return    "Left hand"; break;
	case    11:return   "Right hand"; break;
	case    22:return   "Left thumb"; break;
	case    24:return   "Right thumb"; break;
	default:return "NULL";
	}
}

bool send(int ctl, int x, int y)
{
	//Sleep(50);
	unsigned char data[20] = "";
	data[0] = '#';
	data[1] = ctl;
	data[2] = x / 256;
	data[3] = x % 256;
	data[4] = y / 256;
	data[5] = y % 256;
	data[6] = '*';
	data[7] = 0;
	cout << ctl <<"   "<<x<<"   "<<y <<endl;
	printf("               %d %d %d %d %d %d %d\n ", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
	return serial.SendData(data, 7);

}

void reflashdata()
{
	while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

	myBodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodyArr);
	is_tracking = NO;
	for (int i = 0; i < BODY_COUNT; i++)   //遍历6个人(可能用不完)
	{
		BOOLEAN     result = false;
		rightHandState = HandState_Unknown;
		leftHandState = HandState_Unknown;
		int distance = 0;
		
		if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //判断此人是否被侦测到
		{

			Joint   jointArr[JointType_Count];
			bodyArr[i]->GetJoints(JointType_Count, jointArr);    //获取此人的关节数据
			
			
			//setup部分更新数据
			bodyArr[i]->get_HandLeftState(&leftHandState);
			bodyArr[i]->get_HandRightState(&rightHandState);
			
			cout <<"************"<< leftHandState << "          " << rightHandState << endl;
			//更新关节数据
			for (int j = 0; j < JointType_Count; j++)
			{
				if (jointArr[j].TrackingState == TrackingState_NotTracked) //将确定侦测到的关节显示出来
					continue;
				string  rt = get_name(jointArr[j].JointType);   //获取关节的名字

				//setup部分更新数据
				if (rt == "Head" && jointArr[j].Position.Z * 100 <= bodydepth.max &&jointArr[j].Position.Z * 100 >= bodydepth.min)
						distance=1;


				//跟踪部分更新数据
				if(i==tracked)
				{
					if (rt == "Head" && jointArr[j].Position.Z * 100 <= bodydepth.max &&jointArr[j].Position.Z * 100 >= bodydepth.min)
						is_tracking=YES;
					if (rt == "Left hand")
					{
						lefthand[1] = lefthand[0];
						lefthand[0] = Point3d(jointArr[j].Position.X, jointArr[j].Position.Y, jointArr[j].Position.Z);
					}
					if (rt == "Right hand")
					{
						righthand[1] = righthand[0];
						righthand[0] = Point3d(jointArr[j].Position.X, jointArr[j].Position.Y, jointArr[j].Position.Z);
					}
				}

			}
		}
		//SETUP to runing
		if (distance == 1 && rightHandState == HandState_Closed&& leftHandState == HandState_Closed)
			tracked = i;
	}
	//
	if (myBodyFrame != nullptr)myBodyFrame->Release();

}

void tructbar()
{
	namedWindow("body", 1);
	createTrackbar("人最小距离", "body", &bodydepth.min, 5000);
	createTrackbar("人最大距离", "body", &bodydepth.max, 5000);
	createTrackbar("deta.min", "body", &deta.min, 5000);
	createTrackbar("deta.max", "body", &deta.max, 5000);
	createTrackbar("speed", "body", &speed, 50000);
}

void iniKinect()
{
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);


	mySensor->get_DepthFrameSource(&myDepthSource);
	myDepthSource->OpenReader(&myDepthReader);


	myDepthSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);   //以上为准备好深度数据和骨骼数据的Reader


	img16.create(height, width, CV_16UC1);//为显示深度图像做准备
	img8.create(height, width, CV_8UC1);
}
