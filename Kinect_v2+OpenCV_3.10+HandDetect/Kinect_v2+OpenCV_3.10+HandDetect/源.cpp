#include <iostream>
#include <opencv.hpp>
#include <string>
#include <Kinect.h>
using namespace cv;
using namespace std;
const string  get_name(int n);    //�˺����жϳ��ؽڵ������
string ToGcode(int x, int y, int round);
void tructbar();
void iniKinect();

int myBodyCount = 0, height = 0, width = 0;

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
typedef struct tructvar { int min, max;}tructvar;
tructvar bodydepth;
Point3d lefthand;
Point arduino;
int main(void)
{
	iniKinect();
	tructbar();
    while (1)
    {
        while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
        myDepthFrame->CopyFrameDataToArray(width * height,(UINT16 *)img16.data);
        img16.convertTo(img8,CV_8UC1,255.0 / 4500);
        imshow("Depth Img", img8);  //���ͼ���ת������ʾ

        while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK);

        int myBodyCount = 0;        
        IBody   ** bodyArr = nullptr;
        myBodySource->get_BodyCount(&myBodyCount);
        bodyArr = new IBody *[myBodyCount];
        for (int i = 0; i < myBodyCount; i++)   //bodyArr�ĳ�ʼ��
            bodyArr[i] = nullptr;       

        myBodyFrame->GetAndRefreshBodyData(myBodyCount,bodyArr);
        for (int i = 0; i < myBodyCount; i++)   //����6����(�����ò���)
        {
            BOOLEAN     result = false;
            if (bodyArr[i]->get_IsTracked(&result) == S_OK && result)   //�жϴ����Ƿ���⵽
            {
                cout << "Body " << i << " tracked!" << endl;

                int count   = 0;
                Joint   jointArr[JointType_Count];
                bodyArr[i]->GetJoints(JointType_Count,jointArr);    //��ȡ���˵Ĺؽ�����
                for (int j = 0; j < JointType_Count; j++)
                {
                    if (jointArr[j].TrackingState != TrackingState_Tracked) //��ȷ����⵽�Ĺؽ���ʾ����
                        continue;
                    string  rt = get_name(jointArr[j].JointType);   //��ȡ�ؽڵ�����
                    if (rt != "NULL")   //����ؽ���Ϣ
                    {
                        count++;
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
        if(myDepthFrame!=nullptr)myDepthFrame->Release();
		if (myBodyFrame!= nullptr)myBodyFrame->Release();
        delete[] bodyArr;

        if (waitKey(30) == VK_ESCAPE)
            break;
        //Sleep(1000);    //Ϊ��������ˢ̫�죬ÿ���Ӹ���һ��
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

string ToGcode(int x, int y, int round)
{

	return string();
}

void tructbar()
{
	namedWindow("body",1);
	createTrackbar("����С����", "body", &bodydepth.min, 5000);
	createTrackbar("��������", "body", &bodydepth.max, 5000);
}

void iniKinect()
{
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);
	myBodySource->get_BodyCount(&myBodyCount);


	mySensor->get_DepthFrameSource(&myDepthSource);
	myDepthSource->OpenReader(&myDepthReader);


	myDepthSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&height);
	myDescription->get_Width(&width);   //����Ϊ׼����������ݺ͹������ݵ�Reader


	img16.create(height, width, CV_16UC1);//Ϊ��ʾ���ͼ����׼��
	img8.create(height, width, CV_8UC1);
}
