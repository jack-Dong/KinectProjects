#include"base.h"
HRESULT main()
{
	/*
		����
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
		��ʼ��
	*/
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
		cout << "���Ĭ�ϵ�Kinect�豸ʧ��" << endl;
	}
	else
	{
		cout << "���Ĭ�ϵ�Kinect�豸�ɹ�" << endl;
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
		cout << "û�п��õ�Kinect" << endl;
	}
	else
	{
		cout << "��Kinect�ɹ�" << endl;
	}
	SafeRelease(m_pColorFrameSource);
	/*
	��ȡ����
	*/
	while (true)
	{
		if (!m_pColorFrameReader)
		{
			return S_FALSE;
		}
		//����ÿһ�ζ��ܻ�ȡ��m_pColorFrame����Ϊkinect�Ĳ�ɫ֡��֡���ǹ̶���30����15�����ƹ⣩ ���Ҹտ�ʼʱ�������������������Ĳ��ܱ���������
		hr = m_pColorFrameReader->AcquireLatestFrame(&m_pColorFrame);
		if ( SUCCEEDED(hr) )
		{
			hr = m_pColorFrame->get_FrameDescription(&m_pFrameDescription);
			cout << "��ȡ���ݳɹ�" << endl;
		}
		else
		{
			cout << "                                      ��ȡ����ʧ��" << endl;
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
		SafeRelease(m_pColorFrame); //�ؼ��ĵط�ÿ��Ҫ�ǵ��ͷ�

		//ת����Mat
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
			cout << "����ת������" << endl;
			showImg1 = Mat(cColorHeight / 2, cColorWidth / 2,CV_8UC4,Scalar(0,0,0,0));
		}
		imshow("ColorImg", showImg1);
		waitKey(1000/30);
	}

	/*
		�ͷ� ���������Kinect�ӿڶ�Ӧ���ͷ�
	*/
	//waitKey();

	return S_OK;
}