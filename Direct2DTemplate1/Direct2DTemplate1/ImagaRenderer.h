// ImageRender�� ����ͼ��ͼ����Ⱦ

#pragma once

typedef std::map<std::wstring, ID2D1Bitmap*> BitmapCacheMap;

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
private:
	// ��ʼ�����
	HRESULT								m_hrInit;
	// ���ھ��
	HWND								m_hwnd;
	// D2D ����
	ID2D1Factory*						m_pD2DFactory;
	// WIC ����
	IWICImagingFactory*					m_pWICFactory;
	// DWrite����
	IDWriteFactory*						m_pDWriteFactory;
	// �����ı���Ⱦ��ʽ
	IDWriteTextFormat*					m_pTextFormatMain;
	// D2D��ȾĿ��
	ID2D1HwndRenderTarget*				m_pRenderTarget;
	// ͼ�񻺴�
	BitmapCacheMap						m_mapBitmapCache;
};