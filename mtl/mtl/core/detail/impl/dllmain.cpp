#include "mtl/mtl.hpp"

#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

bool APIENTRY DllMain(HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
{
    (void)hModule;
    (void)lpReserved;
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
