#include "stdafx.h"
#include "included.h"

#define TITLE L"Title"
#define WNDWIDTH 1024
#define WNDHEIGHT 768

#define lengthof(a) sizeof(a)/sizeof(*a)


void SetIdentityMatrix(Matrix4 &mat)
{
    mat.M11 = 1.f; mat.M12 = 0.f; mat.M13 = 0.f; mat.M14 = 0.f;
    mat.M21 = 0.f; mat.M22 = 1.f; mat.M23 = 0.f; mat.M24 = 0.f;
    mat.M31 = 0.f; mat.M32 = 0.f; mat.M33 = 1.f; mat.M34 = 0.f;
    mat.M41 = 0.f; mat.M42 = 0.f; mat.M43 = 0.f; mat.M44 = 1.f;
}

// ����
void ThisApp::ResetReconstruction(){
    m_pReconstruction->ResetReconstruction(&m_worldToCameraTransform, &m_defaultWorldToVolumeTransform);
    //m_pReconstruction->ResetReconstruction(nullptr, nullptr);
}

// ThisApp���캯��
ThisApp::ThisApp(){
    
    // �ؽ�����
    m_reconstructionParams.voxelsPerMeter = 256.f;
    m_reconstructionParams.voxelCountX = 384;
    m_reconstructionParams.voxelCountY = 384; 
    m_reconstructionParams.voxelCountZ = 384; 
    // ʹ��AMP��������
    m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

    // ����Ϊ��λ����
    SetIdentityMatrix(m_worldToCameraTransform);
    SetIdentityMatrix(m_defaultWorldToVolumeTransform);
    // 
    m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
    m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
    m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
    m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
}

// ThisApp��������
ThisApp::~ThisApp(){
    // �����¼�
    if (m_hDepthFrameArrived && m_pDepthFrameReader){
        m_pDepthFrameReader->UnsubscribeFrameArrived(m_hDepthFrameArrived);
        m_hDepthFrameArrived = 0;
    }
    // �ͷ�DepthFrameReader
    SafeRelease(m_pDepthFrameReader);
    // ����Fusion����
    SafeRelease(m_pReconstruction);
    SafeRelease(m_pMapper);
    // ����Fusionͼ��֡
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pSurfaceImageFrame);
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pPointCloud);
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthFloatImage);
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pSmoothDepthFloatImage);
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pNormalImageFrame);
    // ������
    SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);
    SAFE_DELETE_ARRAY(m_pDepthDistortionMap);
    SAFE_DELETE_ARRAY(m_pDepthDistortionLT);
    // ���ŵعر�Kinect
    if (m_pKinect){
        m_pKinect->Close();
    }
    SafeRelease(m_pKinect);
}

// ��ʼ��
HRESULT ThisApp::Initialize(HINSTANCE hInstance, int nCmdShow){
    HRESULT hr = E_FAIL;
    if (SUCCEEDED(static_cast<HRESULT>(m_ImagaRenderer)))
    {
        //register window class
        WNDCLASSEX wcex = { sizeof(WNDCLASSEX) };
        wcex.style = CS_HREDRAW | CS_VREDRAW;
        wcex.lpfnWndProc = ThisApp::WndProc;
        wcex.cbClsExtra = 0;
        wcex.cbWndExtra = sizeof(LONG_PTR);
        wcex.hInstance = hInstance;
        wcex.hCursor = LoadCursorW(nullptr, IDC_ARROW);
        wcex.hbrBackground = nullptr;
        wcex.lpszMenuName = nullptr;
        wcex.lpszClassName = L"Direct2DTemplate";
        wcex.hIcon = nullptr;
        // ע�ᴰ��
        RegisterClassEx(&wcex);
        // ��������
        RECT window_rect = { 0, 0, WNDWIDTH, WNDHEIGHT };
        DWORD window_style = WS_OVERLAPPEDWINDOW;
        AdjustWindowRect(&window_rect, window_style, FALSE);
        AdjustWindowRect(&window_rect, window_style, FALSE);
        window_rect.right -= window_rect.left;
        window_rect.bottom -= window_rect.top;
        window_rect.left = (GetSystemMetrics(SM_CXFULLSCREEN) - window_rect.right) / 2;
        window_rect.top = (GetSystemMetrics(SM_CYFULLSCREEN) - window_rect.bottom) / 2;

        m_hwnd = CreateWindowExW(0, wcex.lpszClassName, TITLE, window_style,
            window_rect.left, window_rect.top, window_rect.right, window_rect.bottom, 0, 0, hInstance, this);
        hr = m_hwnd ? S_OK : E_FAIL;
        if (SUCCEEDED(hr))
        {    
            // ���ô��ھ��
            m_ImagaRenderer.SetHwnd(m_hwnd);
            // ��ʾ����
            ShowWindow(m_hwnd, nCmdShow);
            UpdateWindow(m_hwnd);
        }
    }

    return hr;
}



// ��Ϣѭ��
void ThisApp::RunMessageLoop()
{
    MSG msg;
    HANDLE events[] = { 
        reinterpret_cast<HANDLE>(m_hDepthFrameArrived),
        reinterpret_cast<HANDLE>(m_coordinateMappingChangedEvent),
    };
    while (true){
        // ��Ϣ����
        if (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)){
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        // �����¼�
        // �¼�0: �����֡�¼�
        events[0] = reinterpret_cast<HANDLE>(m_hDepthFrameArrived);
        // �¼�1: ����ӳ��ı��¼�
        events[1] = reinterpret_cast<HANDLE>(m_coordinateMappingChangedEvent);
        // ����¼�
        switch (MsgWaitForMultipleObjects(lengthof(events), events, FALSE, INFINITE, QS_ALLINPUT))
        {
            // events[0]
        case WAIT_OBJECT_0 + 0:
            this->check_depth_frame();
            break;
            // events[1]
        case WAIT_OBJECT_0 + 1:
        {
            int break_point = 9;
        }
            break;
        default:
            break;
        }
        // �˳�
        if (msg.message == WM_QUIT){
            break;
        }
    }
}


// ���ڹ��̺���
LRESULT CALLBACK ThisApp::WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    LRESULT result = 0;

    if (message == WM_CREATE)
    {
        LPCREATESTRUCT pcs = (LPCREATESTRUCT)lParam;
        ThisApp *pOurApp = (ThisApp *)pcs->lpCreateParams;

        ::SetWindowLongPtrW(
            hwnd,
            GWLP_USERDATA,
            PtrToUlong(pOurApp)
            );

        // ����ʼ��Kinect
        if (FAILED(pOurApp->init_kinect())){
            ::MessageBoxW(hwnd, L"��ʼ��Kinect v2ʧ��", L"��ĺ��ź�", MB_ICONERROR);
        }
        result = 1;
    }
    else
    {
        ThisApp *pOurApp = reinterpret_cast<ThisApp *>(static_cast<LONG_PTR>(
            ::GetWindowLongPtrW(
            hwnd,
            GWLP_USERDATA
            )));

        bool wasHandled = false;
        if (pOurApp)
        {
            switch (message)
            {
            case WM_DISPLAYCHANGE:
                InvalidateRect(hwnd, NULL, FALSE);
                result = 0;
                wasHandled = true;
                break;
            case WM_MOUSEWHEEL:
                pOurApp->m_ImagaRenderer.matrix._11 += 0.05f * static_cast<float>(static_cast<short>(HIWORD(wParam))) 
                    / static_cast<float>(WHEEL_DELTA);
                    pOurApp->m_ImagaRenderer.matrix._22 = pOurApp->m_ImagaRenderer.matrix._11;
                pOurApp->m_ImagaRenderer.OnRender();
                break;
            case WM_LBUTTONUP:
                // ����
                pOurApp->ResetReconstruction();
                break;
            case WM_PAINT:
                pOurApp->m_ImagaRenderer.OnRender();
                break;
            case WM_SIZE:
                // �ı䴰�ڴ�С
                pOurApp->m_ImagaRenderer.OnSize(LOWORD(lParam), HIWORD(lParam));
                break;
            case WM_CLOSE:
                // ����β����(�����ȫ�����߳�)��������
                DestroyWindow(hwnd);
                result = 1;
                wasHandled = true;
                break;
            case WM_DESTROY:
                PostQuitMessage(0);
                result = 1;
                wasHandled = true;
                break;
            }
        }

        if (!wasHandled)
        {
            result = DefWindowProc(hwnd, message, wParam, lParam);
        }
    }

    return result;
}


// ��ʼ��Kinect
HRESULT ThisApp::init_kinect(){
    IDepthFrameSource* pDepthFrameSource = nullptr;
    // ���ҵ�ǰĬ��Kinect
    HRESULT hr = ::GetDefaultKinectSensor(&m_pKinect);
    // ��ʿ�ش�Kinect
    if (SUCCEEDED(hr)){
        hr = m_pKinect->Open();
    }
    // ��ȡ���֡Դ(DepthFrameSource)
    if (SUCCEEDED(hr)){
        hr = m_pKinect->get_DepthFrameSource(&pDepthFrameSource);
    }
    // �ٻ�ȡ���֡��ȡ��
    if (SUCCEEDED(hr)){
        hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
    }
    // ע����֡�¼�
    if (SUCCEEDED(hr)){
        hr = m_pDepthFrameReader->SubscribeFrameArrived(&m_hDepthFrameArrived);
    }
    // ��ȡ����ӳ����
    if (SUCCEEDED(hr)) {
        hr = m_pKinect->get_CoordinateMapper(&m_pMapper);
    }
    // ע��ӳ��ı��¼�
    if (SUCCEEDED(hr)) {
        hr = m_pMapper->SubscribeCoordinateMappingChanged(&m_coordinateMappingChangedEvent);
    }
#if 1
    // ��ȡ�豸��Ϣ:: ��������debug
    WCHAR description[MAX_PATH];
    WCHAR instancePath[MAX_PATH];
    UINT memorySize = 0;
    if (SUCCEEDED(hr)){
        hr = NuiFusionGetDeviceInfo(
            m_processorType,
            m_deviceIndex,
            description,
            lengthof(description),
            instancePath,
            lengthof(instancePath),
            &memorySize
            );
        if (hr == E_NUI_BADINDEX){
            ::MessageBoxW(nullptr, L"��ҪDX11֧��", L"����", MB_ICONERROR);
        }
    }
#endif
    // ����Fusion�ݻ��ؽ�
    if (SUCCEEDED(hr)){
        hr = NuiFusionCreateReconstruction(
            &m_reconstructionParams, 
            m_processorType, 
            m_deviceIndex, 
            &m_worldToCameraTransform,
            &m_pReconstruction
            );
        if (hr == E_NUI_GPU_FAIL){
            ::MessageBoxW(nullptr, L"�Կ���֧��Fusion���㣡\n���� ��ʼ��ʧ��", L"����", MB_ICONERROR);
        }
        else if (hr == E_NUI_GPU_OUTOFMEMORY){
            ::MessageBoxW(nullptr, L"�Դ治��", L"����", MB_ICONERROR);
        }
    }
    // �Ȼ�ȡ��ǰ�����絽�ݻ�ת����󲢱��� �����Ժ�ʹ��
    if (SUCCEEDED(hr)){
        hr = m_pReconstruction->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
    }
    // �����������֡
    if (SUCCEEDED(hr)){
        hr = NuiFusionCreateImageFrame(
            NUI_FUSION_IMAGE_TYPE_FLOAT, 
            m_cDepthWidth, 
            m_cDepthHeight, 
            nullptr,
            &m_pDepthFloatImage
            );
    }
    // ����ƽ���������֡
    if (SUCCEEDED(hr)){
        hr = NuiFusionCreateImageFrame(
            NUI_FUSION_IMAGE_TYPE_FLOAT, 
            m_cDepthWidth, 
            m_cDepthHeight,
            nullptr,
            &m_pSmoothDepthFloatImage
            );
    }
    // ��������֡
    if (SUCCEEDED(hr)){
        hr = NuiFusionCreateImageFrame(
            NUI_FUSION_IMAGE_TYPE_POINT_CLOUD,
            m_cDepthWidth,
            m_cDepthHeight,
            nullptr,
            &m_pPointCloud
            );
    }
    // ����Fusionͼ��֡
    if (SUCCEEDED(hr)){
        hr = NuiFusionCreateImageFrame(
            NUI_FUSION_IMAGE_TYPE_COLOR,
            m_cDepthWidth,
            m_cDepthHeight,
            nullptr,
            &m_pSurfaceImageFrame
            );
    }
    // ����Fusion����֡
    if (SUCCEEDED(hr)){
        hr = NuiFusionCreateImageFrame(
            NUI_FUSION_IMAGE_TYPE_COLOR,
            m_cDepthWidth,
            m_cDepthHeight,
            nullptr,
            &m_pNormalImageFrame
            );
    }
    SafeRelease(pDepthFrameSource);
    // ����
    this->ResetReconstruction();
    return hr;
}


// ������֡
void ThisApp::check_depth_frame(){
    if (!m_pDepthFrameReader) return;
#ifdef _DEBUG
    static int frame_num = 0;
    ++frame_num;
    _cwprintf(L"<ThisApp::check_depth_frame>Frame@%8d ", frame_num);
#endif 
    // �����֡�¼�����
    IDepthFrameArrivedEventArgs* pArgs = nullptr;
    // ���֡����
    IDepthFrameReference* pDFrameRef = nullptr;
    // ���֡
    IDepthFrame* pDepthFrame = nullptr;
    // ֡����
    IFrameDescription* pFrameDescription = nullptr;
    // ���֡�������
    int width = 0;
    // ���֡�߶�����
    int height = 0;
    // �����Чֵ
    USHORT depth_min_reliable_distance = 0;
    // ��Զ��Чֵ
    USHORT depth_max_reliable_distance = 0;
    // ֡�����С
    UINT nBufferSize = 0;
    // ��Ȼ���
    UINT16 *pBuffer = nullptr;

    // ��ȡ����
    HRESULT hr = m_pDepthFrameReader->GetFrameArrivedEventData(m_hDepthFrameArrived, &pArgs);
    // ��ȡ����
    if (SUCCEEDED(hr)) {
        hr = pArgs->get_FrameReference(&pDFrameRef);
    }
    // ��ȡ���֡
    if (SUCCEEDED(hr)) {
        hr = pDFrameRef->AcquireFrame(&pDepthFrame);
    }
    // ��ȡ֡����
    if (SUCCEEDED(hr)) {
        hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
    }
    // ��ȡ֡���
    if (SUCCEEDED(hr)) {
        hr = pFrameDescription->get_Width(&width);
    }
    // ��ȡ֡�߶�
    if (SUCCEEDED(hr)) {
        hr = pFrameDescription->get_Height(&height);
    }
    // ��ȡ�����Ч����ֵ
    if (SUCCEEDED(hr)) {
        hr = pDepthFrame->get_DepthMinReliableDistance(&depth_min_reliable_distance);
    }
    // ��ȡ��Զ��Ч����ֵ
    if (SUCCEEDED(hr))  {
        hr = pDepthFrame->get_DepthMaxReliableDistance(&depth_max_reliable_distance);
    }
    // ��ȡ�������
    if (SUCCEEDED(hr))  {
        hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
    }
    // ----------- Fusion ����ʼ
    HRESULT hr4f = hr;
    m_timer.RefreshFrequency();
    m_timer.Start();
    float time__DepthToDepthFloatFrame = 0.0f;
    float time__SmoothDepthFloatFrame = 0.0f;
    float time__ProcessFrame = 0.0f;
    float time__CalculatePointCloud = 0.0f;
    float time__NuiFusionShadePointCloud = 0.0f;
    // ��ԭ������ݹ��측������
    if (SUCCEEDED(hr4f)){
        hr4f = m_pReconstruction->DepthToDepthFloatFrame(
            pBuffer,
            nBufferSize * sizeof(UINT16),
            m_pDepthFloatImage, 
            m_fMinDepthThreshold,
            m_fMaxDepthThreshold,
            true
            );
        time__DepthToDepthFloatFrame = m_timer.DeltaF_ms();
        m_timer.MovStartEnd();
    }
    // ƽ������
    if (SUCCEEDED(hr4f)){
        hr4f = m_pReconstruction->SmoothDepthFloatFrame(
            m_pDepthFloatImage,
            m_pSmoothDepthFloatImage,
            1,
            0.03f
            );
        time__SmoothDepthFloatFrame = m_timer.DeltaF_ms();
        m_timer.MovStartEnd();
    }
    // ����ǰ֡
    if (SUCCEEDED(hr4f)){
        hr4f = m_pReconstruction->ProcessFrame(
            m_pSmoothDepthFloatImage,
            NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
            m_cMaxIntegrationWeight,
            nullptr,
            &m_worldToCameraTransform
            );
        time__ProcessFrame = m_timer.DeltaF_ms();
        m_timer.MovStartEnd();
    }
    // ������
    if (hr4f == E_NUI_FUSION_TRACKING_ERROR){
        m_ImagaRenderer.error_info = L"Fusion����ʧ��, �뱣֤Ŀ���Ǿ�̬��,\n�볣���ؽ�(��������)";
    }
    else if(SUCCEEDED(hr)){
        m_ImagaRenderer.error_info = L"Fusion��������";
    }
    else{
        m_ImagaRenderer.error_info = L"Fusion����ʧ��";
    }
    // ��ȡ��ǰ����
    if (SUCCEEDED(hr4f)){
        Matrix4 calculatedCameraPose;
        hr4f = m_pReconstruction->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
        if (SUCCEEDED(hr4f)){
            m_worldToCameraTransform = calculatedCameraPose;
        }
    }
    // �������
    if (SUCCEEDED(hr4f)){
        hr4f = m_pReconstruction->CalculatePointCloud(
            m_pPointCloud, 
            &m_worldToCameraTransform
            );
        time__CalculatePointCloud = m_timer.DeltaF_ms();
        m_timer.MovStartEnd();
    }
    // ����ͼ��֡
    if (SUCCEEDED(hr4f)){
        // Shading Point Clouid
        Matrix4 worldToBGRTransform = { 0.0f };
        worldToBGRTransform.M11 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountX;
        worldToBGRTransform.M22 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountY;
        worldToBGRTransform.M33 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountZ;
        worldToBGRTransform.M41 = 0.5f;
        worldToBGRTransform.M42 = 0.5f;
        worldToBGRTransform.M43 = 0.0f;
        worldToBGRTransform.M44 = 1.0f;

        //SetIdentityMatrix(worldToBGRTransform);
        //
        hr = NuiFusionShadePointCloud(
            m_pPointCloud, 
            &m_worldToCameraTransform,
            &worldToBGRTransform, 
            m_pSurfaceImageFrame,
            m_pNormalImageFrame
            );
        time__NuiFusionShadePointCloud = m_timer.DeltaF_ms();
        m_timer.MovStartEnd();
    }
    // ����
    swprintf_s(
        m_ImagaRenderer.profiler_info,
        L"  DepthToDepthFloatFrame:   %6.4fms\n"
        L"  SmoothDepthFloatFrame:    %6.4fms\n"
        L"  ProcessFrame:             %6.4fms\n"
        L"  CalculatePointCloud:      %6.4fms\n"
        L"  NuiFusionShadePointCloud: %6.4fms",
        time__DepthToDepthFloatFrame,
        time__SmoothDepthFloatFrame,
        time__ProcessFrame,
        time__CalculatePointCloud,
        time__NuiFusionShadePointCloud
        );
    // ���ӻ�Fusion����
    if (SUCCEEDED(hr4f)){

        // Fusion
        m_ImagaRenderer.WriteBitmapData(
            EnumBitmapIndex::Index_Surface,
            reinterpret_cast<RGBQUAD*>(m_pSurfaceImageFrame->pFrameBuffer->pBits),
            width,
            height
            );
        // Normal
        m_ImagaRenderer.WriteBitmapData(
            EnumBitmapIndex::Index_Normal,
            reinterpret_cast<RGBQUAD*>(m_pNormalImageFrame->pFrameBuffer->pBits),
            width,
            height
            );
    }
    // ������Fusion�������Ļ�
    else if (SUCCEEDED(hr)){

    }
    // ----------- Fusion �������
    // ��������
    if (SUCCEEDED(hr)) {
        auto pRGBXBuffer = m_ImagaRenderer.GetBuffer();
        // �����㷨
        // 0�ź�ɫ (0, min)��128~255������ɫ ����max����128~255��ɫ ֮�����0~255��ɫ
        // ��ͬ��Ƚ�������
        for (UINT i = 0; i < nBufferSize; ++i){
            if (!pBuffer[i]){
                pRGBXBuffer[i].rgbRed = 0xFF;
                pRGBXBuffer[i].rgbGreen = 0;
                pRGBXBuffer[i].rgbBlue = 0;
                pRGBXBuffer[i].rgbReserved = 0xFF;
            }
            else if (pBuffer[i] < depth_min_reliable_distance){
                pRGBXBuffer[i].rgbRed = 0;
                pRGBXBuffer[i].rgbGreen = pBuffer[i] & 0x7F + 0x80;
                pRGBXBuffer[i].rgbBlue = 0;
                pRGBXBuffer[i].rgbReserved = 0xFF;
            }
            else if (pBuffer[i] > depth_max_reliable_distance){
                pRGBXBuffer[i].rgbBlue = pBuffer[i] & 0x7F + 0x80;
                pRGBXBuffer[i].rgbGreen = 0;
                pRGBXBuffer[i].rgbRed = 0;
                pRGBXBuffer[i].rgbReserved = 0xFF;
            }
            else{
                pRGBXBuffer[i].rgbBlue = pBuffer[i] & 0xFF;
                pRGBXBuffer[i].rgbGreen = pRGBXBuffer[i].rgbBlue;
                pRGBXBuffer[i].rgbRed = pRGBXBuffer[i].rgbBlue;
                pRGBXBuffer[i].rgbReserved = 0xFF;
            }
        }
        // ��������
        m_ImagaRenderer.WriteBitmapData(EnumBitmapIndex::Index_Depth, pRGBXBuffer, width, height);
    }
    // ��ȫ�ͷ�
    SafeRelease(pFrameDescription);
    SafeRelease(pDepthFrame);
    SafeRelease(pDFrameRef);
    SafeRelease(pArgs);
#ifdef _DEBUG
    if (SUCCEEDED(hr))
        _cwprintf(L" �ɹ�\n");
    else
        _cwprintf(L" ʧ��\n");
#endif
}