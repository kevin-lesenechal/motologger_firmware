#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <stdarg.h>

#include "tests.hpp"

extern "C" void fatal_error(const char* fmsg, ...)
{
  va_list ap;
  char*   str;

  va_start(ap, fmsg);
  vasprintf(&str, fmsg, ap);
  va_end(ap);
  throw fatal_error_exception(str);
}
