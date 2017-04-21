#include"base.h"
HRESULT main()
{
	/*
		定义
	*/
	IKinectSensor*				m_pKinectSensor = NULL;
	IDepthFrameSource*			m_pDepthFrameSource = NULL;
	IDepthFrameReader *			m_pDepthFrameReader = NULL;
	IDepthFrame *				m_pDepthFrame = NULL;
	RGBQUAD*					m_pDepthRGBX = NULL;
	IFrameDescription *			m_pFrameDescription = NULL;
	ICoordinateMapper*          m_pCoordinateMapper = NULL;
	const int cDepthWidth = 512;
	const int cDepthHeight = 424;
	/*
		初始化
	*/
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (SUCCEEDED(hr))
	{
		cout << "获得默认的Kinect" << endl;
	}
	else
	{
		cout << "没有检测到Kinect设备" << endl;
		return E_FAIL;
	}

	if (m_pKinectSensor != NULL)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			cout << "打开kinect成功" << endl;
		}
		else
		{
			cout << "无法打开kinect" << endl;
			return E_FAIL;
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&m_pDepthFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
	}
	/*
		读取深度数据
	*/
	while (true)
	{
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxReliableDistance = 0;
		UINT nBufferSize = 0;
		UINT16 *pBUffer = NULL;
		hr = m_pDepthFrameReader->AcquireLatestFrame(&m_pDepthFrame);
		if (SUCCEEDED(hr))
		{
			cout << "获取数据成功"<<endl;
			if (SUCCEEDED(hr))
			{
				hr = m_pDepthFrame->get_FrameDescription(&m_pFrameDescription);
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
				hr = m_pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}
			if (SUCCEEDED(hr))
			{
				hr = m_pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
				//对于远的东西我们也想看到
				//nDepthMaxReliableDistance = USHRT_MAX;
			}
			if (SUCCEEDED(hr))
			{
				//这里才是真正的得到了深度数据
				hr = m_pDepthFrame->AccessUnderlyingBuffer(&nBufferSize,&pBUffer);
			}
			//转化成MAT 保证数据的正确转化
			if (m_pDepthRGBX && pBUffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
			{
				//这样简单的显示数据看不出远近的真实差距
				//Mat depth(nHeight,nWidth,CV_16UC1,pBUffer);
				//Mat img;
				//depth.convertTo(img, CV_8U);
				//imshow("Depth_img",depth);
				//imshow("img", img);
				
				for (size_t i = 0; i < nBufferSize; i++)
				{
					USHORT depth = pBUffer[i];
					BYTE intensity = 0;
					if (depth >= nDepthMinReliableDistance && depth <= nDepthMaxReliableDistance)
					{
						//intensity = static_cast<BYTE> (depth % 256);
						intensity = static_cast<BYTE> ( (depth - nDepthMinReliableDistance) /(float)(nDepthMaxReliableDistance- nDepthMinReliableDistance) * 255 );
					}
					m_pDepthRGBX[i].rgbBlue = intensity;
					m_pDepthRGBX[i].rgbGreen = intensity;
					m_pDepthRGBX[i].rgbRed = intensity;
					m_pDepthRGBX[i].rgbReserved = intensity;

				}
				//cout << "nDepthMinReliableDistance = " << nDepthMinReliableDistance << endl; 500
				Mat img(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
				imshow("Depth_imge",img);

				//把深度图像转到相机坐标空间看看
				/*CameraSpacePoint *m_pCameraSpacePiont = new CameraSpacePoint[512 * 424];
				m_pCoordinateMapper->MapDepthFrameToCameraSpace(static_cast<UINT>(nWidth * nHeight), pBUffer, static_cast<UINT>(nWidth * nHeight), m_pCameraSpacePiont);
				Mat CameraLocation(nHeight,nWidth,CV_32FC3,m_pCameraSpacePiont);
				cout << format(CameraLocation,"python") << endl;*/


				//把深度图像转到图像坐标空间看看
				ColorSpacePoint *m_pCorlorSpacePoint = new ColorSpacePoint[512 * 424];
				hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(static_cast<UINT>(nWidth * nHeight), pBUffer, static_cast<UINT>(512 * 424), m_pCorlorSpacePoint);
				if (SUCCEEDED(hr))
				{
					cout << "坐标转化成功 " << endl;
					//Mat ColorLocation(nHeight, nWidth, CV_32FC2, m_pCorlorSpacePoint);
					//cout << format(ColorLocation, "python") << endl;
					Mat depthToColorSpaceImg(1080,1920,CV_8U,Scalar(0,0,0,0));
					for (size_t i = 0; i < nBufferSize; i++)
					{
						UINT row = m_pCorlorSpacePoint[i].Y;
						UINT col = m_pCorlorSpacePoint[i].X;
						if (row >= 0 && row < 1080 && col >= 0 && col < 1920)
						{
							USHORT depth = pBUffer[i];
							BYTE intensity = 0;
							if (depth >= nDepthMinReliableDistance && depth <= nDepthMaxReliableDistance)
							{
								//intensity = static_cast<BYTE> (depth % 256);
								intensity = static_cast<BYTE> ((depth - nDepthMinReliableDistance) / (float)(nDepthMaxReliableDistance - nDepthMinReliableDistance) * 255);
							}
							depthToColorSpaceImg.at<uchar>(row, col) = intensity;
						}
					}
					resize(depthToColorSpaceImg, depthToColorSpaceImg, Size(1920 / 2, 1080 / 2));
					imshow("depthToColorSpaceImg", depthToColorSpaceImg);
				}
				else
				{
					cout << "坐标转化失败 " << endl;
				}
				//waitKey();
			}
		}
		else
		{
			cout << "                                  获取数据失败" << endl;
		}
		SafeRelease(m_pFrameDescription);
		SafeRelease(m_pDepthFrame);
		waitKey(1000/30);
	}

	/*
		释放
	*/
	return S_OK;
}