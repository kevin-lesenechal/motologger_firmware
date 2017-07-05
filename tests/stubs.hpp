#pragma once

#include <functional>
#include <ff.h>

namespace stubs {

extern std::function<FRESULT(FIL*, const TCHAR*, BYTE )>      f_open;
extern std::function<FRESULT(FIL*, const void*, UINT, UINT*)> f_write;
extern std::function<FRESULT(FIL*)>                           f_sync;
extern std::function<FRESULT(FIL*)>                           f_close;

} // ns stubs
