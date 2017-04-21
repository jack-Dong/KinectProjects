#include"base.h"
HRESULT main()
{
	/*
		定义
	*/
	IKinectSensor*        m_pKinectSensor = NULL; 
	IColorFrameSource*    m_pColorFrameSource = NULL;
	IColorFrameReader*    m_pColorFrameReader = NULL;
	IColorFrame*          m_pColorFrame;
	IFrameDescription*    m_pFrameDescription = NULL;

	RGBQUAD*              m_pColorRGBX = NULL;
	static const int      cColorWidth  = 1920;
	static const int      cColorHeight = 1080;

	ColorImageFormat imageFormat = ColorImageFormat_None;
	int nWidth = 0;
	int nHeight = 0;
	UINT nBufferSize = 0;
	RGBQUAD *pBuffer = NULL;

	/*
		初始化
	*/
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
		cout << "获得默认的Kinect设备失败" << endl;
	}
	else
	{
		cout << "获得默认的Kinect设备成功" << endl;
	}

	hr = m_pKinectSensor->Open();
	if (SUCCEEDED(hr))
	{
		hr = m_pKinectSensor->get_ColorFrameSource(&m_pColorFrameSource);
	}

	if (SUCCEEDED(hr))
	{
		hr = m_pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		cout << "没有可用的Kinect" << endl;
	}
	else
	{
		cout << "打开Kinect成功" << endl;
	}
	SafeRelease(m_pColorFrameSource);
	/*
	读取数据
	*/
	while (true)
	{
		if (!m_pColorFrameReader)
		{
			return S_FALSE;
		}
		//不是每一次都能获取到m_pColorFrame，因为kinect的彩色帧的帧率是固定的30或者15（看灯光） 而且刚开始时候过来的数据是有问题的不能被正常解析
		hr = m_pColorFrameReader->AcquireLatestFrame(&m_pColorFrame);
		if ( SUCCEEDED(hr) )
		{
			hr = m_pColorFrame->get_FrameDescription(&m_pFrameDescription);
			cout << "获取数据成功" << endl;
		}
		else
		{
			cout << "                                      获取数据失败" << endl;
		}


		if (SUCCEEDED(hr))
		{
			hr = m_pFrameDescription->get_Width(&nWidth);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pFrameDescription->get_Height(&nHeight);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pColorFrame->get_RawColorImageFormat(&imageFormat);
		}
		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = m_pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
			}
			else if (m_pColorRGBX)
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = m_pColorFrame->CopyConvertedFrameDataToArray(nBufferSize,reinterpret_cast<BYTE*>(pBuffer),ColorImageFormat_Bgra);
			}
			else
			{
				return E_FAIL;
			}
		}
		SafeRelease(m_pColorFrame); //关键的地方每次要记得释放

		//转化成Mat
		Mat showImg1,showImg2;
		if( pBuffer && ( nWidth == cColorWidth) && (nHeight == cColorHeight))
		{
			Mat img1(cColorHeight, cColorWidth, CV_8UC4, pBuffer);
			resize(img1, showImg1, Size(cColorWidth / 2, cColorHeight / 2));
		}
		else
		{
			cout << "pBuffer = " << pBuffer << endl;
			cout << "nWidth = " << nWidth << " " << "nHeight = " << cColorWidth<< endl;
			cout << "nHeight = " << nHeight << " " << "cColorHeight = " << cColorHeight << endl;
			cout << "数据转化有误" << endl;
			showImg1 = Mat(cColorHeight / 2, cColorWidth / 2,CV_8UC4,Scalar(0,0,0,0));
		}
		imshow("ColorImg", showImg1);
		waitKey(1000/30);
	}

	/*
		释放 上面申请的Kinect接口都应该释放
	*/
	//waitKey();

	return S_OK;
}