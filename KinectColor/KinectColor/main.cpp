#include "stdafx.h"
#include "included.h"


// 应用程序入口
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	/*
	所有程序都应该使用HeapSetInformation这个函数并传递HeapEnableTerminationOnCorruption这个选项，让 terminate-on-corruption 这个功能起作用。
	这么做可以确保堆破坏的情况能被注意到，这样就不会造成安全漏洞。这个调用返回的错误码被忽略是因为正确编写的程序能继续正常运行，即使是在这种很少见的失败情况下。
	*/
	HeapSetInformation(NULL, HeapEnableTerminationOnCorruption, NULL, 0);
	#ifdef _DEBUG //在debug模式下开启命令行窗口
		AllocConsole();
		_cwprintf(L"Battle Control  Online! \n");
	#endif
	/*
	Direct2D也是COM组建接口，使用前需要使用 CoInitialize或者CoInitializeEx初始化，不使用时用CoUninitialize反初始化。
	需要注意的是这些函数需要在每个使用COM组件的线程中被调用。
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