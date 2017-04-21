#include"Kinect.h"

//����������4*4�������Ϊ��λ��
void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1.f; mat.M12 = 0.f; mat.M13 = 0.f; mat.M14 = 0.f;
	mat.M21 = 0.f; mat.M22 = 1.f; mat.M23 = 0.f; mat.M24 = 0.f;
	mat.M31 = 0.f; mat.M32 = 0.f; mat.M33 = 1.f; mat.M34 = 0.f;
	mat.M41 = 0.f; mat.M42 = 0.f; mat.M43 = 0.f; mat.M44 = 1.f;
}



Kinect::Kinect()
{
	//�����ؽ�����
	this->m_reconstrucionPrams.voxelsPerMeter = 256.f;
	this->m_reconstrucionPrams.voxelCountX = 384;
	this->m_reconstrucionPrams.voxelCountY = 384;
	this->m_reconstrucionPrams.voxelCountZ = 384;

	//ʹ��GPU���м��� AMP����
	this->m_processType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

	//������ת����������Ϊ��λ����
	SetIdentityMatrix(this->m_wordToCameratransform);
	SetIdentityMatrix(this->m_wordToVolumeTransform);

	//��������Ĳ���
	this->m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
	this->m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
	this->m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
	this->m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
}

Kinect::~Kinect()
{
	

}

//�����ؽ�
void Kinect::ResetReconstruction()
{
	this->m_pReconstruction->ResetReconstruction(&this->m_wordToCameratransform, &this->m_wordToVolumeTransform);
}

//��ʼ��Kinect
HRESULT Kinect::Init_kinect()
{
	IDepthFrameSource *pDepthFrameSource = NULL;
	//����Ĭ�ϵ�Kinect
	HRESULT hr = GetDefaultKinectSensor(&this->m_pKinectSensor);
	//��Kinect
	if (SUCCEEDED(hr))
	{
		hr = this->m_pKinectSensor->Open();
	}
	//��ȡ�������Դ
	if (SUCCEEDED(hr))
	{
		cout << "��ȡ��Kinect���ɹ���" << endl;
		hr = this->m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	}
	else
	{
		cout << "��Kinectʧ��" << endl;
	}
	//��ȡ��ȶ�ȡ��
	if (SUCCEEDED(hr))
	{
		hr = pDepthFrameSource->OpenReader(&this->m_pDepthFrameReader);
	}
	
	//����Fusion�����ؽ�
	if (SUCCEEDED(hr))
	{
		hr = NuiFusionCreateReconstruction(
			&this->m_reconstrucionPrams,
			this->m_processType,
			this->m_deviceIndex,
			&this->m_wordToCameratransform,
			&this->m_pReconstruction
			);
		//����һЩ���ܵĴ���
		if (hr == E_NUI_GPU_FAIL)
		{
			cout << "�Կ���֧��Fusion������ʼ��ʧ�ܣ�" <<endl;
			return S_FALSE;
		}
		else if(hr == E_NUI_GPU_OUTOFMEMORY)
		{
			cout << "�Դ治��" << endl;
			return S_FALSE;
		}
	}
	//�Ȼ�ȡ��ǰ�����絽���ص�ת�����󲢱��棬�����Ժ�ʹ��
	if (SUCCEEDED(hr))
	{
		hr = this->m_pReconstruction->GetCurrentWorldToVolumeTransform(&this->m_wordToVolumeTransform);
	}
	//�����������֡
	if (SUCCEEDED(hr))
	{
		hr = NuiFusionCreateImageFrame(
			NUI_FUSION_IMAGE_TYPE_FLOAT,
			this->m_cDepthWidth,
			this->m_cDepthheight,
			nullptr,
			&this->m_pDepthFloatImage
			);
	}
	//����ƽ���������֡
	if (SUCCEEDED(hr))
	{
		hr = NuiFusionCreateImageFrame(
			NUI_FUSION_IMAGE_TYPE_FLOAT,
			this->m_cDepthWidth,
			this->m_cDepthheight,
			nullptr,
			&this->m_pSmoothedDepthFloatImage
			);
	}
	//��������֡
	if (SUCCEEDED(hr))
	{
		hr = NuiFusionCreateImageFrame(
			NUI_FUSION_IMAGE_TYPE_POINT_CLOUD ,
			this->m_cDepthWidth,
			this->m_cDepthheight,
			nullptr,
			&this->m_pPointCloud
			);
	}
	//����Fusion����ͼ֡
	if (SUCCEEDED(hr))
	{
		hr = NuiFusionCreateImageFrame(
			NUI_FUSION_IMAGE_TYPE_COLOR,
			this->m_cDepthWidth,
			this->m_cDepthheight,
			nullptr,
			&this->m_pSurfaceImage
			);
	}
	//����Fusion����֡
	if (SUCCEEDED(hr))
	{
		hr = NuiFusionCreateImageFrame(
			NUI_FUSION_IMAGE_TYPE_COLOR,
			this->m_cDepthWidth,
			this->m_cDepthheight,
			nullptr,
			&this->m_pNormalImage
			);
	}
	SafeRelease(pDepthFrameSource);
	//����
	this->ResetReconstruction();
	return S_OK;
}

//����
HRESULT Kinect::Update()
{
	//���֡
	IDepthFrame *pDepthFrame  =  NULL;
	//֡������
	IFrameDescription *pFrameDescription = NULL;
	//���֡���
	int width = 0;
	//���֡�߶�
	int height = 0;
	//�����Чֵ
	USHORT depthMinReliableDistance = 0;
	//��Զ��Ч����
	USHORT depthMaxReliableDistance = 0;
	//֡�����С
	UINT nBufferSize = 0;
	//��Ȼ���
	UINT16 *pBuffer = NULL;

	//-------------------��ʼ��ȡ����

	//��ȡ���֡
	HRESULT hr = this->m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	//��ȡ֡������
	if (SUCCEEDED(hr))
	{
		cout << "��ȡ���ݳɹ�" << endl;
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);

		//��ȡ֡���
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&width);
		}
		//��ȡ֡�߶�
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&height);
		}
		//��ȡ�����Ч����
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&depthMinReliableDistance);
		}
		//��ȡ��Զ��Ч����
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMaxReliableDistance(&depthMaxReliableDistance);
		}
		//��ȡ�������
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize,&pBuffer);
		}
		//��ʾ�������
		if (pBuffer && (this->m_cDepthWidth == width) && (this->m_cDepthheight == height))
		{
			Mat img(height,width,CV_16U,pBuffer);
			imshow("�������",img);
			//waitKey(1);
	
		
		}

		//----------------Fusion ��ʼ����

		//��ԭ������ݹ��측������
		if (SUCCEEDED(hr))
		{
			hr = this->m_pReconstruction->DepthToDepthFloatFrame(
				pBuffer,
				nBufferSize * sizeof(UINT16),
				this->m_pDepthFloatImage,
				this->m_fMinDepthThreshold,
				this->m_fMaxDepthThreshold,
				true
				);
		}
		//ƽ���������
		if (SUCCEEDED(hr))
		{
			hr = this->m_pReconstruction->SmoothDepthFloatFrame(
				this->m_pDepthFloatImage,
				this->m_pSmoothedDepthFloatImage,
				1,
				0.03f
				);
		}
		//����ǰ֡
		if (SUCCEEDED(hr))
		{
			hr = m_pReconstruction->ProcessFrame(
				this->m_pSmoothedDepthFloatImage,
				NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
				this->m_cMaxIntergrationWeight,
				NULL,
				&this->m_wordToCameratransform
				);
			//���һЩ����
			if (hr == E_NUI_FUSION_TRACKING_ERROR)
			{
				cout << "Fusion ����ʧ�ܣ���֤Ŀ���Ǿ�̬�� ���¿�ʼ�ؽ�" << endl;
				this->ResetReconstruction();
			}
			else if (SUCCEEDED(hr))
			{
				cout << "Fusion ��������" << endl;
			}
			else
			{
				cout << "Fusion ����ʧ�ܣ�ԭ����" << endl;
			}
		}

		//��ȡ��ǰ����
		if (SUCCEEDED(hr))
		{
			hr = this->m_pReconstruction->GetCurrentWorldToCameraTransform(&this->m_wordToCameratransform);
		}
		//�������
		if (SUCCEEDED(hr))
		{
			hr = this->m_pReconstruction->CalculatePointCloud(
				this->m_pPointCloud,
				&this->m_wordToCameratransform
				);
		}
		//����ͼ��֡
		if (SUCCEEDED(hr))
		{
			// Shading Point Clouid
			Matrix4 worldToBGRTransform = { 0.0f };
			worldToBGRTransform.M11 = this->m_reconstrucionPrams.voxelsPerMeter / this->m_reconstrucionPrams.voxelCountX;
			worldToBGRTransform.M22 = this->m_reconstrucionPrams.voxelsPerMeter / this->m_reconstrucionPrams.voxelCountY;
			worldToBGRTransform.M33 = this->m_reconstrucionPrams.voxelsPerMeter / this->m_reconstrucionPrams.voxelCountZ;
			worldToBGRTransform.M41 = 0.5f;
			worldToBGRTransform.M42 = 0.5f;
			worldToBGRTransform.M43 = 0.0f;
			worldToBGRTransform.M44 = 1.0f;

			hr = NuiFusionShadePointCloud(
				this->m_pPointCloud,
				&this->m_wordToCameratransform,
				&worldToBGRTransform,
				this->m_pSurfaceImage,
				this->m_pNormalImage
				);
		}
		//��ʾFusion����
		if (SUCCEEDED(hr))
		{
			//����ͼ��������ʾһ��
			Mat cloudPointImg(height, width, CV_8UC4, reinterpret_cast<RGBQUAD *>(this->m_pPointCloud->pFrameBuffer->pBits));
			imshow("ֱ�ӵ�������", cloudPointImg);

			Mat SurfaceImg(height,width,CV_8UC4,reinterpret_cast<RGBQUAD *>(this->m_pSurfaceImage->pFrameBuffer->pBits));
			imshow("Fusi��������",SurfaceImg);

			Mat NormalImg(height, width, CV_8UC4, reinterpret_cast<RGBQUAD *>(this->m_pNormalImage->pFrameBuffer->pBits));
			imshow("Fusion��������", NormalImg);
		}
		waitKey(1000/30);
		//------------Fution �������
	}
	else
	{
		cout << "                     ��ȡ����ʧ��" << endl;
	}

	//�ͷŵ�ǰ֡
	SafeRelease(pDepthFrame);
	return hr;
}