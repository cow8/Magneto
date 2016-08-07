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
const string  get_name(int n);    //�˺����жϳ��ؽڵ������
bool send(int ctl, int x, int y);
void reflashdata();
void tructbar();
void iniKinect();
int is_san();
int height = 0, width = 0;
int speed = 1000,yuzhi=3;
int is_tracking = NO;
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
tructvar bodydepth = { 100,150 }, deta = {2,20};
IBody   * bodyArr[BODY_COUNT];
Point3d lefthand[2], righthand[2], nohand = { 100,100,100 };
Point arduino;
CSerial serial;
int tracked;
///////////////////////////////////////
int main()
{
	lefthand[0] = lefthand[1] = nohand;
	if (serial.OpenSerialPort(_T("COM2:"), 9600, 8, 1) == false) return -1;
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
		}
		cout << "ok"<<endl;
		send(CENTER, 0, 0);
		arduino = Point(0, 0);
		reflashdata();

		while (is_tracking)
		{
			int detax=0, detay=0;
			detax+= (lefthand[0].x - lefthand[1].x)*speed;
			detay+= (lefthand[0].y - lefthand[1].y)*speed;
			detax+= (righthand[0].x - righthand[1].x)*speed;
			detay+= (righthand[0].y - righthand[1].y)*speed;
			if (abs(detax) >= deta.max || abs(detax) <=deta.min) detax = 0;
			if (abs(detay) >= deta.max || abs(detay) <= deta.min) detay = 0;
			arduino.x += detax;//���ٶ���
			arduino.y += detay;
			cout << arduino;
			if ((righthand[0].z - righthand[1].z < -0.05 && rightHandState!=HandState_Open)|| (lefthand[0].z - lefthand[1].z < -0.05 && leftHandState != HandState_Open))
				send(ROUND, arduino.x, arduino.y),cout<<"     ROUND"<<endl;
			else
				send(NORMAL, arduino.x, arduino.y);
			reflashdata();//��дIS_HUMAN
			if (waitKey(30) == VK_ESCAPE)
				break;
		}

		if (waitKey(30) == VK_ESCAPE)
			break;
		//Sleep(100);    //Ϊ��������ˢ̫�죬ÿ���Ӹ���һ��

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

void reflashdata()
{
	while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

	myBodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodyArr);
	is_tracking = NO;
	for (int i = 0; i < BODY_COUNT; i++)   //����6����(�����ò���)
	{
		BOOLEAN     result = false;
		rightHandState = HandState_Unknown;
		leftHandState = HandState_Unknown;
		int distance = 0;
		
		if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //�жϴ����Ƿ���⵽
		{

			Joint   jointArr[JointType_Count];
			bodyArr[i]->GetJoints(JointType_Count, jointArr);    //��ȡ���˵Ĺؽ�����
			
			
			//setup���ָ�������
			bodyArr[i]->get_HandLeftState(&leftHandState);
			bodyArr[i]->get_HandRightState(&rightHandState);
			
			cout <<"************"<< leftHandState << "          " << rightHandState << endl;
			//���¹ؽ�����
			for (int j = 0; j < JointType_Count; j++)
			{
				if (jointArr[j].TrackingState == TrackingState_NotTracked) //��ȷ����⵽�Ĺؽ���ʾ����
					continue;
				string  rt = get_name(jointArr[j].JointType);   //��ȡ�ؽڵ�����

				//setup���ָ�������
				if (rt == "Head" && jointArr[j].Position.Z * 100 <= bodydepth.max &&jointArr[j].Position.Z * 100 >= bodydepth.min)
						distance=1;


				//���ٲ��ָ�������
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
	createTrackbar("����С����", "body", &bodydepth.min, 5000);
	createTrackbar("��������", "body", &bodydepth.max, 5000);
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
	myDescription->get_Width(&width);   //����Ϊ׼����������ݺ͹������ݵ�Reader


	img16.create(height, width, CV_16UC1);//Ϊ��ʾ���ͼ����׼��
	img8.create(height, width, CV_8UC1);
}
