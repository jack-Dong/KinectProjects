// ImageRender�� ����ͼ��ͼ����Ⱦ

#pragma once

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424

enum class EnumBitmapIndex{
    Index_Depth = 0,    // ���ͼ��
    Index_Surface,      // ����
    Index_Normal,       // ����
};

class ImageRenderer{
public:
    // ���캯��
    ImageRenderer();
    // ��������
    ~ImageRenderer();
    // ��Ⱦ
    HRESULT OnRender();
    // ���ô��ھ��
    void SetHwnd(HWND hwnd){ m_hwnd = hwnd; }
    // �ı��С
    void OnSize(UINT width, UINT height){
        if (!m_pRenderTarget)return;
        D2D1_SIZE_U size = {
            width, height
        };
        m_pRenderTarget->Resize(size);
    }
    // ���س�ʼ�����
    operator HRESULT() const{ return m_hrInit; }
    // ��ȡ����
    __forceinline RGBQUAD* GetBuffer(){ return m_pColorRGBX; }
    // д������
    void WriteBitmapData(EnumBitmapIndex, RGBQUAD*,int, int);
private:
    // �����豸�޹���Դ
    HRESULT CreateDeviceIndependentResources();
    // �����豸�й���Դ
    HRESULT CreateDeviceResources();
    // �����豸�й���Դ
    void DiscardDeviceResources();
    // ���ļ���ȡλͼ
    HRESULT LoadBitmapFromFile(ID2D1RenderTarget*, IWICImagingFactory *, PCWSTR uri, UINT, UINT, ID2D1Bitmap **);
public:
    // D2Dת������
    D2D1_MATRIX_3X2_F                   matrix = D2D1::Matrix3x2F::Identity();
private:
    // ��ʼ�����
    HRESULT                             m_hrInit = E_FAIL;
    // ���ھ��
    HWND                                m_hwnd = nullptr;
    // D2D ����
    ID2D1Factory*                       m_pD2DFactory = nullptr;
    // WIC ����
    IWICImagingFactory*                 m_pWICFactory = nullptr;
    // DWrite����
    IDWriteFactory*                     m_pDWriteFactory = nullptr;
    // �����ı���Ⱦ��ʽ
    IDWriteTextFormat*                  m_pTextFormatMain = nullptr;
    // D2D��ȾĿ��
    ID2D1HwndRenderTarget*              m_pRenderTarget = nullptr;
    // ֡��������
    RGBQUAD*                            m_pColorRGBX = nullptr;
    // ����λͼ֡
    ID2D1Bitmap*                        m_pDrawBitmap = nullptr;
    // ����λͼ֡
    ID2D1Bitmap*                        m_pSurfaceBitmap = nullptr;
    // ����λͼ֡
    ID2D1Bitmap*                        m_pNormalBitmap = nullptr;
    // ��ɫ��ˢ
    ID2D1Brush*                         m_pMainBrush = nullptr;
    // ��ʱ��
    PrecisionTimer                      m_timer;
    // FPS
    FLOAT                               m_fFPS = 0.f;
public:
    // ������Ϣ
    const WCHAR*                        error_info = L"";
    // ������Ϣ
    WCHAR                               profiler_info[2048];
};