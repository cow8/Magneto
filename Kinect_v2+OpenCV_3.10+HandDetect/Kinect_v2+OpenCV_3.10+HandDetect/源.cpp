#include <iostream>
#include <opencv.hpp>
#include <cstring>
#include <Kinect.h>
#include <windows.h>
#include "Serial.h"
#include "Serial.cpp"
#define NORMAL 0
#define ZERO 1
#define CENTER 2
#define ROUND 3
#define STOP 4
using namespace cv;
using namespace std;
const string  get_name(int n);    //此函数判断出关节点的名字
bool send(int ctl,int x, int y);
void work();
void tructbar();
void iniKinect();
int height = 0, width = 0;
int speed=0;
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
typedef struct tructvar { int min, max;}tructvar;
tructvar bodydepth;
Point3d lefthand;
Point3d zerolefthand;
Point arduino;
CSerial serial;
int main(void)
{
	
	//if(serial.OpenSerialPort(_T("COM2:"), 9600, 8, 1)==false) return -1;
	iniKinect();
	tructbar();
	//while(send(STOP, 0, 0)) Sleep(300);
    while (1)
    {
        //while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
        //myDepthFrame->CopyFrameDataToArray(width * height,(UINT16 *)img16.data);
        //img16.convertTo(img8,CV_8UC1,255.0 / 4500);
        //imshow("Depth Img", img8);  //深度图像的转化及显示

        while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

        //int myBodyCount = 0;        
        //IBody   ** bodyArr = nullptr;
        //myBodySource->get_BodyCount(&myBodyCount);
        //bodyArr = new IBody *[myBodyCount];
        //for (int i = 0; i < myBodyCount; i++)   //bodyArr的初始化
         //   bodyArr[i] = nullptr;       
		IBody   * bodyArr[BODY_COUNT] = {0};

        myBodyFrame->GetAndRefreshBodyData(BODY_COUNT,bodyArr);
        for (int i = 0; i < BODY_COUNT; i++)   //遍历6个人(可能用不完)
        {
            BOOLEAN     result = false;
            if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //判断此人是否被侦测到
            {
                cout << "Body " << i << " tracked!" << endl;
                Joint   jointArr[JointType_Count];
                bodyArr[i]->GetJoints(JointType_Count,jointArr);    //获取此人的关节数据
				
				//判断距离合法否
				if (!(jointArr[0].Position.Z <= bodydepth.max &&jointArr[0].Position.Z >= bodydepth.min)) continue;

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

                        cout << "   " << rt << " tracked" << endl;
						if (rt == "Left hand")
						{
							lefthand = Point3d((int)(jointArr[j].Position.X * 1000000), (int)(jointArr[j].Position.Y * 1000000), (int)(jointArr[j].Position.Z * 1000));
							cout << "lefthand" << lefthand.x << "   " << lefthand.y << "   "<< lefthand.z<<endl;
						}
                           
                    }
                }
            }
        }
		work();
		

		
        if(myDepthFrame!=nullptr)myDepthFrame->Release();
		if (myBodyFrame!= nullptr)myBodyFrame->Release();

        if (waitKey(30) == VK_ESCAPE)
            break;
        //Sleep(1000);    //为避免数据刷太快，每秒钟更新一次
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
    default :return "NULL";
    }
}

bool send(int ctl,int x, int y)
{
	char data[20]="";
	data[0]= '#';
	data[1]= ctl;
	data[2]= (x / 256)+1;
	data[3]= (x % 256)+1;
	data[4]= (y / 256)+1;
	data[5]= (y % 256)+1;
	data[6]= '*';
	data[7] = '*';
	return serial.SendData(data, 8);
	
}

void work()
{
	////////////ini
	if (rightHandState == HandState_Closed && leftHandState == HandState_Closed)
	{
		send(CENTER, 0, 0);
		arduino = Point(11000, 16000);
		zerolefthand = lefthand;
	}
	////////////
	if (leftHandState == HandState_NotTracked)
	{
		send(ZERO, 0, 0);
		arduino = Point(0, 0);
	}
	//////////////
	send(NORMAL, (lefthand.x - zerolefthand.x)*speed, (lefthand.y - zerolefthand.y)*speed);
}

void tructbar()
{
	namedWindow("body",1);
	createTrackbar("人最小距离", "body", &bodydepth.min, 5000);
	createTrackbar("人最大距离", "body", &bodydepth.max, 5000);
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

