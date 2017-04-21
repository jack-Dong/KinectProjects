#pragma once

#include"base.h"

class Kinect
{
public:
	//初始化类
	Kinect();
	//释放资源
	~Kinect();
	//初始化Kinect
	HRESULT Init_kinect();
	//更新
	HRESULT Update();
	//重置
	void ResetReconstruction();
private:
	//下面开始定义这个类需要的参数   真的很多
	//Kinect v2传感器
	IKinectSensor*						m_pKinectSensor = NULL;
	//深度读取器
	IDepthFrameReader*					m_pDepthFrameReader = NULL;
	//Kinect Fusion 容积重建
	INuiFusionReconstruction*			m_pReconstruction = NULL;
	//平滑前的浮点深度帧
	NUI_FUSION_IMAGE_FRAME*				m_pDepthFloatImage = NULL;
	//平滑后的浮点深度帧
	NUI_FUSION_IMAGE_FRAME*				m_pSmoothedDepthFloatImage = NULL;
	//点云FUSION图像帧
	NUI_FUSION_IMAGE_FRAME*				m_pPointCloud = NULL;
	//表面Fusion 图像帧
	NUI_FUSION_IMAGE_FRAME*				m_pSurfaceImage = NULL;
	//法线 Fusion图像帧
	NUI_FUSION_IMAGE_FRAME*				m_pNormalImage = NULL;
	//kinect Fusion世界到相机转换
	Matrix4								m_wordToCameratransform;
	//Kinect Fusion 世界到容积转换
	Matrix4								m_wordToVolumeTransform;
	//Kinect 容积重建参数
	NUI_FUSION_RECONSTRUCTION_PARAMETERS		m_reconstrucionPrams;
	//Kinect Fusion 相机参数
	NUI_FUSION_CAMERA_PARAMETERS		m_cameraParameters;
	//图像宽度
	UINT								m_cDepthWidth = NUI_DEPTH_RAW_WIDTH;
	//图像高度
	UINT								m_cDepthheight = NUI_DEPTH_RAW_HEIGHT;
	//FUsion处理器类型
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processType;
	//设备索引 （处理设备使用默认的-1）
	int									m_deviceIndex = -1;
	//最大多少临时的数据参加重建
	USHORT								m_cMaxIntergrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;
	//深度最近阈值
	float								m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;
	//深度最远阈值
	float								m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;
};

