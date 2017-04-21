#include "stdafx.h"
#include "included.h"


// Ӧ�ó������
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	HeapSetInformation(NULL, HeapEnableTerminationOnCorruption, NULL, 0);
#ifdef _DEBUG
	AllocConsole();
	_cwprintf(L"Battle Control  Online! \n");
#endif

	if (SUCCEEDED(CoInitialize(NULL)))
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