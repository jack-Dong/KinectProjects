// ThisApp�� ������ĳ���

#pragma once

class ThisApp
{
public:
	// ���캯��
	ThisApp(){};
	// ��������
	~ThisApp(){};
	// ��ʼ��
	HRESULT Initialize(HINSTANCE hInstance, int nCmdShow);
	// ��Ϣѭ��
	void RunMessageLoop();
private:
	// ���ڹ��̺���
	static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
private:
	// ���ھ��
	HWND			m_hwnd = nullptr;
	// ��Ⱦ��
	ImageRenderer	m_ImagaRenderer;
};