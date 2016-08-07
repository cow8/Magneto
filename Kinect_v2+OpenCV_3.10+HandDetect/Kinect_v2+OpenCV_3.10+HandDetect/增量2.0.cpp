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
typedef struct tructvar { int min, max; }tructvar;
const string  get_name(int n);    //此函数判断出关节点的名字
bool send(int ctl, int x, int y);
void reflashcameradata();
void tructbar();
void iniKinect();
int is_san();
int height = 0, width = 0;
int speed = 1000, yuzhi = 3;
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
////////////////////////////////////

tructvar bodydepth = { 100,150 }, deta = { 2,20 };
IBody   * bodyArr[BODY_COUNT];
Point3d lefthand[2], righthand[2], nohand = { 100,100,100 };
Point arduino;
CSerial serial;

///////////////////////////////////////
int main()
{
	//lefthand[0] = lefthand[1] = nohand;
	//if (serial.OpenSerialPort(_T("COM2:"), 9600, 8, 1) == false) return -1;
	iniKinect();
	tructbar();
	//waitKey(30);
	while (1)
	{
		int tracked=-1;
		while (1)//堵塞去等待setup的人
		{
			int breakflag = 0;
			HandState leftHandState, rightHandState;
			leftHandState = HandState_Unknown;
			rightHandState = HandState_Unknown;
			while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);
			myBodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodyArr);

			for (int i = 0; i < BODY_COUNT; i++)
			{
				BOOLEAN     result = false;
				leftHandState = HandState_Unknown;
				rightHandState = HandState_Unknown;
				int distance=0;
				if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)
				{
					bodyArr[i]->get_HandLeftState(&leftHandState);
					bodyArr[i]->get_HandRightState(&rightHandState);//获得手的数据
					
					Joint   jointArr[JointType_Count];
					bodyArr[i]->GetJoints(JointType_Count, jointArr);
					for (int j = 0; j < JointType_Count; j++)
					{
						if (jointArr[j].TrackingState == TrackingState_NotTracked) //将确定侦测到的关节显示出来
							continue;
						string  rt = get_name(jointArr[j].JointType);   //获取关节的名字
						if (rt == "Head" && jointArr[j].Position.Z * 100 <= bodydepth.max &&jointArr[j].Position.Z * 100 >= bodydepth.min)
							distance = 1;
					}
				}
				if (leftHandState == HandState_Closed && rightHandState == HandState_Closed && distance==1)
				{
					tracked = i;
					breakflag = 1;
					break;
				}

			}
			if (myBodyFrame != nullptr)myBodyFrame->Release();
			if (breakflag == 1) break;
			cout << "waiting" << endl;
			//send(SLEEP, 0, 0);
		}
		
		cout << "Tracked:   " << tracked << endl;
		bodyArr[tracked]
		while (1)
		{
			while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);
			myBodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodyArr);

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

#ifdef f
void reflashcameradata()
{
	while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

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
					if (rt == "Head" && jointArr[j].Position.Z * 100 <= bodydepth.max &&jointArr[j].Position.Z * 100 >= bodydepth.min)
						is_human = YES;
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
	}
	//
	if (myBodyFrame != nullptr)myBodyFrame->Release();

}
#endif
void tructbar()
{
	namedWindow("body", 1);
	createTrackbar("人最小距离", "body", &bodydepth.min, 5000);
	createTrackbar("人最大距离", "body", &bodydepth.max, 5000);
	createTrackbar("deta.min", "body", &deta.min, 50);
	createTrackbar("deta.max", "body", &deta.max, 500);
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
