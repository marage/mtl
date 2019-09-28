#include "mtl/mtl.hpp"

#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN             //  从 Windows 头文件中排除极少使用的信息
// Windows 头文件:
#include <Windows.h>

bool APIENTRY DllMain(HANDLE /*hModule*/, DWORD  ul_reason_for_call, LPVOID /*lpReserved*/)
{
  if (ul_reason_for_call == DLL_PROCESS_ATTACH) {
    //WSADATA wsd;
    //if (SOCKET_ERROR == WSAStartup(MAKEWORD(2, 2), &wsd))				//socket
    //	return FALSE;
  } else if (ul_reason_for_call == DLL_PROCESS_DETACH) {
    //WSACleanup();
  }
  return TRUE;
}

#endif
