#include "stdafx.h"
#include "included.h"


// Ӧ�ó������
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	/*
	���г���Ӧ��ʹ��HeapSetInformation�������������HeapEnableTerminationOnCorruption���ѡ��� terminate-on-corruption ������������á�
	��ô������ȷ�����ƻ�������ܱ�ע�⵽�������Ͳ�����ɰ�ȫ©����������÷��صĴ����뱻��������Ϊ��ȷ��д�ĳ����ܼ����������У���ʹ�������ֺ��ټ���ʧ������¡�
	*/
	HeapSetInformation(NULL, HeapEnableTerminationOnCorruption, NULL, 0);
	#ifdef _DEBUG //��debugģʽ�¿��������д���
		AllocConsole();
		_cwprintf(L"Battle Control  Online! \n");
	#endif
	/*
	Direct2DҲ��COM�齨�ӿڣ�ʹ��ǰ��Ҫʹ�� CoInitialize����CoInitializeEx��ʼ������ʹ��ʱ��CoUninitialize����ʼ����
	��Ҫע�������Щ������Ҫ��ÿ��ʹ��COM������߳��б����á�
	*/
	if ( SUCCEEDED(CoInitialize(NULL)) )
	{
		{
			ThisApp app;
			if (SUCCEEDED(app.Initialize(hInstance, nCmdShow)))
			{
				app.RunMessageLoop();
			}
		}
		CoUninitialize();
	}

	#ifdef _DEBUG
		_cwprintf(L"Battle Control Terminated! \n");
		FreeConsole();
	#endif

	return 0;
}