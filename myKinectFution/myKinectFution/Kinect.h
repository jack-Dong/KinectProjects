#pragma once

#include"base.h"

class Kinect
{
public:
	//��ʼ����
	Kinect();
	//�ͷ���Դ
	~Kinect();
	//��ʼ��Kinect
	HRESULT Init_kinect();
	//����
	HRESULT Update();
	//����
	void ResetReconstruction();
private:
	//���濪ʼ�����������Ҫ�Ĳ���   ��ĺܶ�
	//Kinect v2������
	IKinectSensor*						m_pKinectSensor = NULL;
	//��ȶ�ȡ��
	IDepthFrameReader*					m_pDepthFrameReader = NULL;
	//Kinect Fusion �ݻ��ؽ�
	INuiFusionReconstruction*			m_pReconstruction = NULL;
	//ƽ��ǰ�ĸ������֡
	NUI_FUSION_IMAGE_FRAME*				m_pDepthFloatImage = NULL;
	//ƽ����ĸ������֡
	NUI_FUSION_IMAGE_FRAME*				m_pSmoothedDepthFloatImage = NULL;
	//����FUSIONͼ��֡
	NUI_FUSION_IMAGE_FRAME*				m_pPointCloud = NULL;
	//����Fusion ͼ��֡
	NUI_FUSION_IMAGE_FRAME*				m_pSurfaceImage = NULL;
	//���� Fusionͼ��֡
	NUI_FUSION_IMAGE_FRAME*				m_pNormalImage = NULL;
	//kinect Fusion���絽���ת��
	Matrix4								m_wordToCameratransform;
	//Kinect Fusion ���絽�ݻ�ת��
	Matrix4								m_wordToVolumeTransform;
	//Kinect �ݻ��ؽ�����
	NUI_FUSION_RECONSTRUCTION_PARAMETERS		m_reconstrucionPrams;
	//Kinect Fusion �������
	NUI_FUSION_CAMERA_PARAMETERS		m_cameraParameters;
	//ͼ����
	UINT								m_cDepthWidth = NUI_DEPTH_RAW_WIDTH;
	//ͼ��߶�
	UINT								m_cDepthheight = NUI_DEPTH_RAW_HEIGHT;
	//FUsion����������
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processType;
	//�豸���� �������豸ʹ��Ĭ�ϵ�-1��
	int									m_deviceIndex = -1;
	//��������ʱ�����ݲμ��ؽ�
	USHORT								m_cMaxIntergrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;
	//��������ֵ
	float								m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;
	//�����Զ��ֵ
	float								m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;
};

