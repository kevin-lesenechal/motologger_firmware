#pragma once

#include <functional>
#include <stdexcept>
#include <ff.h>

namespace stubs {

  extern std::function<FRESULT(FIL*, const TCHAR*, BYTE )>      f_open;
  extern std::function<FRESULT(FIL*, const void*, UINT, UINT*)> f_write;
  extern std::function<FRESULT(FIL*)>                           f_sync;
  extern std::function<FRESULT(FIL*)>                           f_close;

} // ns stubs

struct fatal_error_exception : std::runtime_error
{
  fatal_error_exception(const std::string& err)
    : std::runtime_error("fatal_error(): " + err)
  {}
};

#define EXPECT_FATAL_ERROR(expr) EXPECT_THROW(expr, fatal_error_exception)
