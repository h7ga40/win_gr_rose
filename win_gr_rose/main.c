#include <windows.h>
#include <tchar.h>

int main()
{
	int result;
	HANDLE gr_rose;
	int (*device_main)(void);

	//gr_rose = LoadLibrary(_T("usb_host"));
	gr_rose = LoadLibrary(_T("usb_device"));
	if (gr_rose == NULL)
		return -1;

	device_main = (int (*)(void))GetProcAddress(gr_rose, "device_main");
	if (device_main == NULL)
		return -1;

	result = device_main();

	FreeLibrary(gr_rose);

	return result;
}
