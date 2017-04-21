#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <Kinect.h>
#include <NuiKinectFusionApi.h>

#pragma comment(lib,"Kinect20.lib")
#pragma comment(lib,"Kinect20.Fusion.lib")

//������ģʽ�����ļ���Ȼ�������
#ifdef _DEBUG
#pragma comment(lib,"opencv_core249d.lib")
#pragma comment(lib,"opencv_highgui249d.lib")
#pragma comment(lib,"opencv_imgproc249d.lib")
#endif

#ifndef _DEBUG
#pragma comment(lib,"opencv_core249.lib")
#pragma comment(lib,"opencv_highgui249.lib")
#pragma comment(lib,"opencv_imgproc249.lib")
#endif

using namespace std;
using namespace cv;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

