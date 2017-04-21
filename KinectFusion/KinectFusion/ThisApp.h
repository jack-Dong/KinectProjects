// ThisApp�� ������ĳ���

#pragma once

// ThisApp��
class ThisApp{
    // ���������곤 ����
    typedef NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE PROCESSOR_TYPE;
public:
    // ���캯��
    ThisApp();
    // ��������
    ~ThisApp();
    // ��ʼ��
    HRESULT Initialize(HINSTANCE hInstance, int nCmdShow);
    // ��Ϣѭ��
    void RunMessageLoop();
    // ����
    void ResetReconstruction();
private:
    // ���ڹ��̺���
    static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
    // ��ʼ��Kinect
    HRESULT init_kinect();
    // ������֡
    void check_depth_frame();
private:
    // ���ھ��
    HWND                                    m_hwnd = nullptr;
    // Kinect v2 ������
    IKinectSensor*                          m_pKinect = nullptr;
    // ���֡��ȡ��
    IDepthFrameReader*                      m_pDepthFrameReader = nullptr;
    // Kinect Fusion �ݻ��ؽ�
    INuiFusionReconstruction*               m_pReconstruction = nullptr;
    // ��Ȼ���
    UINT16*                                 m_pDepthImagePixelBuffer = nullptr;
    // ƽ��ǰ�ĸ������֡
    NUI_FUSION_IMAGE_FRAME*                 m_pDepthFloatImage = nullptr;
    // ƽ����ĸ������֡
    NUI_FUSION_IMAGE_FRAME*                 m_pSmoothDepthFloatImage = nullptr;
    // ���� Fusion ͼ��֡
    NUI_FUSION_IMAGE_FRAME*                 m_pPointCloud = nullptr;
    // ���� Fusion ͼ��֡
    NUI_FUSION_IMAGE_FRAME*                 m_pSurfaceImageFrame = nullptr;
    // ���� Fusion ͼ��֡
    NUI_FUSION_IMAGE_FRAME*                 m_pNormalImageFrame = nullptr;
    // ӳ����
    ICoordinateMapper*                      m_pMapper = nullptr;
    // ��ȿռ��
    DepthSpacePoint*                        m_pDepthDistortionMap = nullptr;
    // ���ʧ��
    UINT*                                   m_pDepthDistortionLT = nullptr;
    // ��ɫ��֡�¼� ������nullptr��ʼ�� ����
    WAITABLE_HANDLE                         m_hDepthFrameArrived = 0;
    // ����ӳ��ı��¼�
    WAITABLE_HANDLE                         m_coordinateMappingChangedEvent = 0;
    // Kinect Fusion ���ת��
    Matrix4                                 m_worldToCameraTransform;
    // Ĭ��Kinect Fusion ���絽�ݻ�ת��
    Matrix4                                 m_defaultWorldToVolumeTransform;
    // Kinect Fusion �ݻ��ؽ�����
    NUI_FUSION_RECONSTRUCTION_PARAMETERS    m_reconstructionParams;
    // Kinect Fusion �������
    NUI_FUSION_CAMERA_PARAMETERS            m_cameraParameters;
    // ��Ⱦ��
    ImageRenderer                           m_ImagaRenderer;
    // ͼ����
    UINT                                    m_cDepthWidth = NUI_DEPTH_RAW_WIDTH;
    // ͼ��߶�
    UINT                                    m_cDepthHeight = NUI_DEPTH_RAW_HEIGHT;
    // Fusion����������
    ThisApp::PROCESSOR_TYPE                 m_processorType;
    // �豸���� �����豸ʹ��Ĭ�ϵ�(-1)
    int                                     m_deviceIndex = -1;
    //
    USHORT                                  m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;
    // ����
    USHORT                                  unused = 0;
    // ��ʱ��
    PrecisionTimer                          m_timer;
    // ��������ֵ
    float                                   m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;
    // �����Զ��ֵ
    float                                   m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;
};