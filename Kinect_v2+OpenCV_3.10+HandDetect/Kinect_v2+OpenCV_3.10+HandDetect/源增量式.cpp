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
void reflashcameradata();
void tructbar();
void iniKinect();
int is_san();
int height = 0, width = 0;
int speed =1000;
int is_human = NO;
int is_setup = NO;
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
tructvar bodydepth = {100,150};
IBody   * bodyArr[BODY_COUNT];
Point3d lefthand[2], righthand[2], nohand = {100,100,100};
Point arduino;
CSerial serial;

///////////////////////////////////////
int main()
{
	lefthand[0] = lefthand[1] = nohand; 
	if (serial.OpenSerialPort(_T("COM2:"), 9600, 8, 1) == false) return -1;
	iniKinect();
	tructbar();

	while (1)
	{
		while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
		myDepthFrame->CopyFrameDataToArray(width * height, (UINT16 *)img16.data);
		img16.convertTo(img8, CV_8UC1, 255.0 / 4500);
		imshow("Depth Img", img8);  //深度图像的转化及显示
									//
		reflashcameradata();
		while (!(rightHandState == HandState_Closed && leftHandState == HandState_Closed))
		{
			reflashcameradata();
			send(SLEEP, 0, 0);
		}
		





		if(is_human==NO)
		{
			lefthand[0] = lefthand[1] = nohand;
			cout << "nohuman"<<endl;
			arduino = { 0,0 };
			send(ZERO,0,0);
		}
		else
		{
			
			cout << "yeshuman" << endl;
			
			arduino.x += abs(lefthand[1].x - lefthand[0].x)>1? lefthand[1].x - lefthand[0].x:0;
			arduino.y += abs(lefthand[1].y - lefthand[0].y)>1 ? lefthand[1].y - lefthand[0].y : 0;
			cout <<arduino<< endl;
			if(rightHandState==HandState_Closed && righthand[1].z*100<=100)
				send(ROUND, arduino.x, arduino.y);
			else send(NORMAL, arduino.x, arduino.y);
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
	char data[20] = "";
	data[0] = '#';
	data[1] = ctl;
	data[2] = (x / 256) + 1;
	data[3] = (x % 256) + 1;
	data[4] = (y / 256) + 1;
	data[5] = (y % 256) + 1;
	data[6] = '*';
	data[7] = 0;
	//cout << ctl <<"   "<<x<<"   "<<y <<endl;
	return serial.SendData(data, strlen(data));

}

void reflashcameradata()
{
	while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

	//bodyArr[BODY_COUNT] = { 0 };
	myBodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodyArr);
	is_human = NO;
	for (int i = 0; i < BODY_COUNT; i++)   //遍历6个人(可能用不完)
	{
		BOOLEAN     result = false;
		if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //判断此人是否被侦测到
		{
			//cout << "Body " << i << " tracked!" << endl;
			Joint   jointArr[JointType_Count];
			bodyArr[i]->GetJoints(JointType_Count, jointArr);    //获取此人的关节数据

			leftHandState = HandState_Unknown;
			bodyArr[i]->get_HandLeftState(&leftHandState);
			rightHandState = HandState_Unknown;
			bodyArr[i]->get_HandRightState(&rightHandState);//更新左右手数据


			for (int j = 0; j < JointType_Count; j++)
			{
				if (jointArr[j].TrackingState == TrackingState_NotTracked) //将确定侦测到的关节显示出来
					continue;
				string  rt = get_name(jointArr[j].JointType);   //获取关节的名字
				if (rt != "NULL")   //输出关节信息
				{

					//cout << "   " << rt << " tracked" << endl;
					if(rt == "Left hand" && jointArr[j].Position.Z * 100 <= bodydepth.max &&jointArr[j].Position.Z * 100 >= bodydepth.min)
					is_human = YES;
					if (rt == "Left hand")
					{
						if(lefthand[0]!=nohand)
						{
							lefthand[0] = lefthand[1];
							lefthand[1] = Point3d(jointArr[j].Position.X*speed, jointArr[j].Position.Y*speed, jointArr[j].Position.Z);
						}
						else
						{
							lefthand[0] = Point3d(jointArr[j].Position.X*speed, jointArr[j].Position.Y*speed, jointArr[j].Position.Z);
							lefthand[1] = Point3d(jointArr[j].Position.X*speed, jointArr[j].Position.Y*speed, jointArr[j].Position.Z);
						}
					}
					if (rt == "Right hand")
					{
						if (righthand[0] != nohand)
						{
							righthand[0] = righthand[1];
							righthand[1] = Point3d(jointArr[j].Position.X*speed, jointArr[j].Position.Y*speed, jointArr[j].Position.Z);
						}
						else
						{
							righthand[0] = Point3d(jointArr[j].Position.X*speed, jointArr[j].Position.Y*speed, jointArr[j].Position.Z);
							righthand[1] = Point3d(jointArr[j].Position.X*speed, jointArr[j].Position.Y*speed, jointArr[j].Position.Z);
						}
					}

				}
			}
		}
	}
	if (myDepthFrame != nullptr)myDepthFrame->Release();
	if (myBodyFrame != nullptr)myBodyFrame->Release();

}

void tructbar()
{
	namedWindow("body", 1);
	createTrackbar("人最小距离", "body", &bodydepth.min, 5000);
	createTrackbar("人最大距离", "body", &bodydepth.max, 5000);
	createTrackbar("speed", "body", &speed, 5000);
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
