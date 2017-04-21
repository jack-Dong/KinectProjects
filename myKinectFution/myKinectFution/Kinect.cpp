#include"Kinect.h"

//方便是设置4*4放入矩阵为单位阵
void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1.f; mat.M12 = 0.f; mat.M13 = 0.f; mat.M14 = 0.f;
	mat.M21 = 0.f; mat.M22 = 1.f; mat.M23 = 0.f; mat.M24 = 0.f;
	mat.M31 = 0.f; mat.M32 = 0.f; mat.M33 = 1.f; mat.M34 = 0.f;
	mat.M41 = 0.f; mat.M42 = 0.f; mat.M43 = 0.f; mat.M44 = 1.f;
}



Kinect::Kinect()
{
	//设置重建参数
	this->m_reconstrucionPrams.voxelsPerMeter = 256.f;
	this->m_reconstrucionPrams.voxelCountX = 384;
	this->m_reconstrucionPrams.voxelCountY = 384;
	this->m_reconstrucionPrams.voxelCountZ = 384;

	//使用GPU进行计算 AMP加速
	this->m_processType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

	//把两个转换矩阵设置为单位矩阵
	SetIdentityMatrix(this->m_wordToCameratransform);
	SetIdentityMatrix(this->m_wordToVolumeTransform);

	//设置相机的参数
	this->m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
	this->m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
	this->m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
	this->m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
}

Kinect::~Kinect()
{
	

}

//重置重建
void Kinect::ResetReconstruction()
{
	this->m_pReconstruction->ResetReconstruction(&this->m_wordToCameratransform, &this->m_wordToVolumeTransform);
}

//初始化Kinect
HRESULT Kinect::Init_kinect()
{
	IDepthFrameSource *pDepthFrameSource = NULL;
	//查找默认的Kinect
	HRESULT hr = GetDefaultKinectSensor(&this->m_pKinectSensor);
	//打开Kinect
	if (SUCCEEDED(hr))
	{
		hr = this->m_pKinectSensor->Open();
	}
	//获取到深度资源
	if (SUCCEEDED(hr))
	{
		cout << "获取到Kinect并成功打开" << endl;
		hr = this->m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	}
	else
	{
		cout << "打开Kinect失败" << endl;
	}
	//获取深度读取器
	if (SUCCEEDED(hr))
	{
		hr = pDepthFrameSource->OpenReader(&this->m_pDepthFrameReader);
	}
	
	//创建Fusion体素重建
	if (SUCCEEDED(hr))
	{
		hr = NuiFusionCreateReconstruction(
			&this->m_reconstrucionPrams,
			this->m_processType,
			this->m_deviceIndex,
			&this->m_wordToCameratransform,
			&this->m_pReconstruction
			);
		//处理一些可能的错误
		if (hr == E_NUI_GPU_FAIL)
		{
			cout << "显卡不支持Fusion计算或初始化失败！" <<endl;
			return S_FALSE;
		}
		else if(hr == E_NUI_GPU_OUTOFMEMORY)
		{
			cout << "显存不足" << endl;
			return S_FALSE;
		}
	}
	//先获取当前的世界到体素的转换矩阵并保存，方便以后使用
	if (SUCCEEDED(hr))
	{
		hr = this->m_pReconstruction->GetCurrentWorldToVolumeTransform(&this->m_wordToVolumeTransform);
	}
	//创建浮点深度帧
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
	//创建平滑浮点深度帧
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
	//创建点云帧
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
	//创建Fusion表面图帧
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
	//创建Fusion法线帧
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
	//重置
	this->ResetReconstruction();
	return S_OK;
}

//更新
HRESULT Kinect::Update()
{
	//深度帧
	IDepthFrame *pDepthFrame  =  NULL;
	//帧描述符
	IFrameDescription *pFrameDescription = NULL;
	//深度帧宽度
	int width = 0;
	//深度帧高度
	int height = 0;
	//最近有效值
	USHORT depthMinReliableDistance = 0;
	//最远有效距离
	USHORT depthMaxReliableDistance = 0;
	//帧缓存大小
	UINT nBufferSize = 0;
	//深度缓存
	UINT16 *pBuffer = NULL;

	//-------------------开始获取参数

	//获取深度帧
	HRESULT hr = this->m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	//获取帧描述符
	if (SUCCEEDED(hr))
	{
		cout << "获取数据成功" << endl;
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);

		//获取帧宽度
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&width);
		}
		//获取帧高度
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&height);
		}
		//获取最近有效距离
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&depthMinReliableDistance);
		}
		//获取最远有效距离
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMaxReliableDistance(&depthMaxReliableDistance);
		}
		//获取深度数据
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize,&pBuffer);
		}
		//显示深度数据
		if (pBuffer && (this->m_cDepthWidth == width) && (this->m_cDepthheight == height))
		{
			Mat img(height,width,CV_16U,pBuffer);
			imshow("深度数据",img);
			//waitKey(1);
	
		
		}

		//----------------Fusion 开始处理

		//由原深度数据构造浮点数据
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
		//平滑深度数据
		if (SUCCEEDED(hr))
		{
			hr = this->m_pReconstruction->SmoothDepthFloatFrame(
				this->m_pDepthFloatImage,
				this->m_pSmoothedDepthFloatImage,
				1,
				0.03f
				);
		}
		//处理当前帧
		if (SUCCEEDED(hr))
		{
			hr = m_pReconstruction->ProcessFrame(
				this->m_pSmoothedDepthFloatImage,
				NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
				this->m_cMaxIntergrationWeight,
				NULL,
				&this->m_wordToCameratransform
				);
			//检查一些错误
			if (hr == E_NUI_FUSION_TRACKING_ERROR)
			{
				cout << "Fusion 跟踪失败，保证目标是静态的 重新开始重建" << endl;
				this->ResetReconstruction();
			}
			else if (SUCCEEDED(hr))
			{
				cout << "Fusion 跟踪正常" << endl;
			}
			else
			{
				cout << "Fusion 跟踪失败，原因不明" << endl;
			}
		}

		//获取当前矩阵
		if (SUCCEEDED(hr))
		{
			hr = this->m_pReconstruction->GetCurrentWorldToCameraTransform(&this->m_wordToCameratransform);
		}
		//计算点云
		if (SUCCEEDED(hr))
		{
			hr = this->m_pReconstruction->CalculatePointCloud(
				this->m_pPointCloud,
				&this->m_wordToCameratransform
				);
		}
		//生成图像帧
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
		//显示Fusion数据
		if (SUCCEEDED(hr))
		{
			//点云图像拿来显示一下
			Mat cloudPointImg(height, width, CV_8UC4, reinterpret_cast<RGBQUAD *>(this->m_pPointCloud->pFrameBuffer->pBits));
			imshow("直接点云数据", cloudPointImg);

			Mat SurfaceImg(height,width,CV_8UC4,reinterpret_cast<RGBQUAD *>(this->m_pSurfaceImage->pFrameBuffer->pBits));
			imshow("Fusi表面数据",SurfaceImg);

			Mat NormalImg(height, width, CV_8UC4, reinterpret_cast<RGBQUAD *>(this->m_pNormalImage->pFrameBuffer->pBits));
			imshow("Fusion法线数据", NormalImg);
		}
		waitKey(1000/30);
		//------------Fution 处理结束
	}
	else
	{
		cout << "                     获取数据失败" << endl;
	}

	//释放当前帧
	SafeRelease(pDepthFrame);
	return hr;
}