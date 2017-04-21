#include"Kinect.h"
HRESULT main()
{
	Kinect kinect;
	HRESULT hr =  kinect.Init_kinect();
	while (true)
	{
		hr = kinect.Update();
	}
	return hr;
}