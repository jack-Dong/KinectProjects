// ImageRender�� ����ͼ��ͼ����Ⱦ

#pragma once

typedef std::map<std::wstring, ID2D1Bitmap*> BitmapCacheMap;
#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 1080

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
	// ���س�ʼ�����
	operator HRESULT() const{ return m_hrInit; }
    // ��ȡ����
    __forceinline RGBQUAD* GetBuffer(){ return m_pColorRGBX; }
    // д������
    void WriteBitmapData(RGBQUAD*, int, int);
private:
	// ��ȡͼƬ
	// bitmapName	[in] : �ļ���
	// ����: NULL��ʾʧ�� �������Ϊλͼ��ָ��
	ID2D1Bitmap* GetBitmap(std::wstring& bitmapName);
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
    HRESULT								m_hrInit = E_FAIL;
	// ���ھ��
    HWND								m_hwnd = nullptr;
	// D2D ����
    ID2D1Factory*						m_pD2DFactory = nullptr;
	// WIC ����
    IWICImagingFactory*					m_pWICFactory = nullptr;
	// DWrite����
    IDWriteFactory*						m_pDWriteFactory = nullptr;
	// �����ı���Ⱦ��ʽ
    IDWriteTextFormat*					m_pTextFormatMain = nullptr;
	// D2D��ȾĿ��
    ID2D1HwndRenderTarget*				m_pRenderTarget = nullptr;
    // ֡��������
    RGBQUAD*                            m_pColorRGBX = nullptr;
    // ����λͼ֡
    ID2D1Bitmap*                        m_pDrawBitmap = nullptr;
    // ��ɫ��ˢ
    ID2D1Brush*                         m_pWhiteBrush = nullptr;
    // ��ʱ��
    PrecisionTimer                      m_timer;
    // FPS
    FLOAT                               m_fFPS = 0.f;
	// ͼ�񻺴�
	BitmapCacheMap						m_mapBitmapCache;
};