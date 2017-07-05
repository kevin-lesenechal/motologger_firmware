#include "tests.hpp"

std::function<FRESULT(FIL*, const TCHAR*, BYTE )>      stubs::f_open  = nullptr;
std::function<FRESULT(FIL*, const void*, UINT, UINT*)> stubs::f_write = nullptr;
std::function<FRESULT(FIL*)>                           stubs::f_sync  = nullptr;
std::function<FRESULT(FIL*)>                           stubs::f_close = nullptr;

extern "C" FRESULT f_open(FIL* fp, const TCHAR* path, BYTE mode)
{
  return stubs::f_open(fp, path, mode);
}

extern "C" FRESULT f_write(FIL* fp, const void* buff, UINT btw, UINT* bw)
{
  return stubs::f_write(fp, buff, btw, bw);
}

extern "C" FRESULT f_sync(FIL* fp)
{
  return stubs::f_sync(fp);
}

extern "C" FRESULT f_close(FIL* fp)
{
  return stubs::f_close(fp);
}
