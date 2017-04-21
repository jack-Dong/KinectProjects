#include "stdafx.h"
#include "included.h"
#include <wchar.h>


// ImageRender�๹�캯��
ImageRenderer::ImageRenderer(){
    // ������Դ
    m_hrInit = CreateDeviceIndependentResources();
    // ����������
    m_pColorRGBX = new RGBQUAD[DEPTH_WIDTH*DEPTH_HEIGHT];
    if (!m_pColorRGBX) m_hrInit = E_OUTOFMEMORY;
    m_timer.Start();
    //
    *this->profiler_info = 0;
}


// �����豸�޹���Դ
HRESULT ImageRenderer::CreateDeviceIndependentResources(){
    HRESULT hr = S_OK;

    // ���� Direct2D ����.
    hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

    if (SUCCEEDED(hr))
    {
        // ���� WIC ����.
        hr = CoCreateInstance(
            CLSID_WICImagingFactory,
            NULL,
            CLSCTX_INPROC_SERVER,
            IID_IWICImagingFactory,
            reinterpret_cast<void **>(&m_pWICFactory)
            );
    }

    if (SUCCEEDED(hr))
    {
        // ���� DirectWrite ����.
        hr = DWriteCreateFactory(
            DWRITE_FACTORY_TYPE_SHARED,
            __uuidof(m_pDWriteFactory),
            reinterpret_cast<IUnknown **>(&m_pDWriteFactory)
            );
    }

    if (SUCCEEDED(hr))
    {
        // ���������ı���ʽ.
        hr = m_pDWriteFactory->CreateTextFormat(
            L"SimHei",
            nullptr,
            DWRITE_FONT_WEIGHT_NORMAL,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            50.f,
            L"", //locale
            &m_pTextFormatMain
            );
    }

    return hr;
}

// ���ļ���ȡλͼ
HRESULT ImageRenderer::LoadBitmapFromFile(
    ID2D1RenderTarget *pRenderTarget,
    IWICImagingFactory *pIWICFactory,
    PCWSTR uri,
    UINT destinationWidth,
    UINT destinationHeight,
    ID2D1Bitmap **ppBitmap
    )
{
    IWICBitmapDecoder *pDecoder = NULL;
    IWICBitmapFrameDecode *pSource = NULL;
    IWICStream *pStream = NULL;
    IWICFormatConverter *pConverter = NULL;
    IWICBitmapScaler *pScaler = NULL;

    HRESULT hr = pIWICFactory->CreateDecoderFromFilename(
        uri,
        NULL,
        GENERIC_READ,
        WICDecodeMetadataCacheOnLoad,
        &pDecoder
        );

    if (SUCCEEDED(hr))
    {
        hr = pDecoder->GetFrame(0, &pSource);
    }
    if (SUCCEEDED(hr))
    {
        hr = pIWICFactory->CreateFormatConverter(&pConverter);
    }


    if (SUCCEEDED(hr))
    {
        if (destinationWidth != 0 || destinationHeight != 0)
        {
            UINT originalWidth, originalHeight;
            hr = pSource->GetSize(&originalWidth, &originalHeight);
            if (SUCCEEDED(hr))
            {
                if (destinationWidth == 0)
                {
                    FLOAT scalar = static_cast<FLOAT>(destinationHeight) / static_cast<FLOAT>(originalHeight);
                    destinationWidth = static_cast<UINT>(scalar * static_cast<FLOAT>(originalWidth));
                }
                else if (destinationHeight == 0)
                {
                    FLOAT scalar = static_cast<FLOAT>(destinationWidth) / static_cast<FLOAT>(originalWidth);
                    destinationHeight = static_cast<UINT>(scalar * static_cast<FLOAT>(originalHeight));
                }

                hr = pIWICFactory->CreateBitmapScaler(&pScaler);
                if (SUCCEEDED(hr))
                {
                    hr = pScaler->Initialize(
                        pSource,
                        destinationWidth,
                        destinationHeight,
                        WICBitmapInterpolationModeCubic
                        );
                }
                if (SUCCEEDED(hr))
                {
                    hr = pConverter->Initialize(
                        pScaler,
                        GUID_WICPixelFormat32bppPBGRA,
                        WICBitmapDitherTypeNone,
                        NULL,
                        0.f,
                        WICBitmapPaletteTypeMedianCut
                        );
                }
            }
        }
        else
        {
            hr = pConverter->Initialize(
                pSource,
                GUID_WICPixelFormat32bppPBGRA,
                WICBitmapDitherTypeNone,
                NULL,
                0.f,
                WICBitmapPaletteTypeMedianCut
                );
        }
    }
    if (SUCCEEDED(hr))
    {
        hr = pRenderTarget->CreateBitmapFromWicBitmap(
            pConverter,
            NULL,
            ppBitmap
            );
    }

    SafeRelease(pDecoder);
    SafeRelease(pSource);
    SafeRelease(pStream);
    SafeRelease(pConverter);
    SafeRelease(pScaler);

    return hr;
}

// �����豸�����Դ
HRESULT ImageRenderer::CreateDeviceResources()
{
    HRESULT hr = S_OK;
    if (!m_pRenderTarget)
    {
        RECT rc;
        GetClientRect(m_hwnd, &rc);

        D2D1_SIZE_U size = D2D1::SizeU(
            rc.right - rc.left,
            rc.bottom - rc.top
            );

        // ���� Direct2D RenderTarget.
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            D2D1::RenderTargetProperties(),
            D2D1::HwndRenderTargetProperties(m_hwnd, size),
            &m_pRenderTarget
            );
        D2D1_BITMAP_PROPERTIES prop = { { DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_PREMULTIPLIED }, 96.f, 96.f };

        // �����̻�λͼ
        if (SUCCEEDED(hr)){
            hr = m_pRenderTarget->CreateBitmap(
                D2D1::SizeU(DEPTH_WIDTH, DEPTH_HEIGHT),
                prop,
                &m_pDrawBitmap
                );
        }
        // ����Surfaceλͼ
        if (SUCCEEDED(hr)){
            hr = m_pRenderTarget->CreateBitmap(
                D2D1::SizeU(DEPTH_WIDTH, DEPTH_HEIGHT),
                prop,
                &m_pSurfaceBitmap
                );
        }
        // ��������λͼ
        if (SUCCEEDED(hr)){
            hr = m_pRenderTarget->CreateBitmap(
                D2D1::SizeU(DEPTH_WIDTH, DEPTH_HEIGHT),
                prop,
                &m_pNormalBitmap
                );
        }
        // ��������ˢ
        if (SUCCEEDED(hr)){
            ID2D1SolidColorBrush* pSolidColorBrush = nullptr;
            hr = m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::White), &pSolidColorBrush);
            m_pMainBrush = pSolidColorBrush;
        }
    }

    return hr;
}

// ImageRender��������
ImageRenderer::~ImageRenderer(){
    DiscardDeviceResources();
    SafeRelease(m_pD2DFactory);
    SafeRelease(m_pWICFactory);
    SafeRelease(m_pDWriteFactory);
    SafeRelease(m_pTextFormatMain);
    SAFE_DELETE_ARRAY(m_pColorRGBX);
}

// �����豸�����Դ
void ImageRenderer::DiscardDeviceResources(){
    // ���λͼ����
    SafeRelease(m_pDrawBitmap);
    SafeRelease(m_pSurfaceBitmap);
    SafeRelease(m_pNormalBitmap);
    SafeRelease(m_pMainBrush);
    SafeRelease(m_pRenderTarget);
}

// ��Ⱦͼ��ͼ��
HRESULT ImageRenderer::OnRender(){
    HRESULT hr = S_OK;
    WCHAR buffer[1024];
    D2D1_RECT_F rect;
    // ���Դ�����Դ
    hr = CreateDeviceResources();
    if (SUCCEEDED(hr)){
        // ��ʼ
        m_pRenderTarget->BeginDraw();
        // ����ת��
        m_pRenderTarget->SetTransform(this->matrix);
        // ����
        m_pRenderTarget->Clear(D2D1::ColorF(0x0066CCFF));
        // ��ʽ�̻�.........
        rect = { 0, 0, static_cast<float>(DEPTH_WIDTH), static_cast<float>(DEPTH_HEIGHT) };
        m_pRenderTarget->DrawBitmap(m_pDrawBitmap, &rect);
        rect = { static_cast<float>(DEPTH_WIDTH), 0, static_cast<float>(DEPTH_WIDTH * 2), static_cast<float>(DEPTH_HEIGHT) };
        m_pRenderTarget->DrawBitmap(m_pSurfaceBitmap, &rect);
        rect = { static_cast<float>(DEPTH_WIDTH*2), 0, static_cast<float>(DEPTH_WIDTH *3), static_cast<float>(DEPTH_HEIGHT) };
        m_pRenderTarget->DrawBitmap(m_pNormalBitmap, &rect);
        // ��λ��ʾFPS
        auto length = swprintf_s(
            buffer, 
            L"֡��: %2.2f\n�Ŵ���x: %2.2f\n�Ŵ���y: %2.2f\nFusion��ʱ:\n%s\n%s",
            m_fFPS, 
            this->matrix._11, 
            this->matrix._22,
            this->profiler_info,
            this->error_info
            );
        auto size = m_pRenderTarget->GetSize();
        rect.left = 0.f; rect.right = size.width;
        rect.top = 0.f; rect.bottom = size.height;
        m_pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity());
        m_pRenderTarget->DrawText(buffer, length, m_pTextFormatMain, &rect, m_pMainBrush);
        // �����̻�
        hr = m_pRenderTarget->EndDraw();
        // �յ��ؽ���Ϣʱ���ͷ���Դ���ȴ��´��Զ�����
        if (hr == D2DERR_RECREATE_TARGET) {
            DiscardDeviceResources();
            hr = S_OK;
        }
    }
    return hr;
}

// д������
void ImageRenderer::WriteBitmapData(EnumBitmapIndex index, RGBQUAD* buffer, int width, int height){
    D2D1_RECT_U rect = { 0, 0, width, height };;
    switch (index)
    {
    case EnumBitmapIndex::Index_Depth: // ���ͼ��
        if (m_pDrawBitmap){
            m_pDrawBitmap->CopyFromMemory(&rect, buffer, width*sizeof(RGBQUAD));
        }
        m_fFPS = 1000.f / m_timer.DeltaF_ms();
        m_timer.MovStartEnd();
        OnRender();
        break;
    case EnumBitmapIndex::Index_Surface:
        if (m_pSurfaceBitmap){
            m_pSurfaceBitmap->CopyFromMemory(&rect, buffer, width*sizeof(RGBQUAD));
        }
        break;
    case EnumBitmapIndex::Index_Normal:
        if (m_pNormalBitmap){
            m_pNormalBitmap->CopyFromMemory(&rect, buffer, width*sizeof(RGBQUAD));
        }
        break;
    }
}