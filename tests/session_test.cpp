#include <gtest/gtest.h>
#include <ff.h>
#include <motologger/session.hpp>

#include "tests.hpp"

using motologger::Session;

TEST(Session, open)
{
  bool fopen_stub_called = false;
  stubs::f_open = [&](FIL*, const TCHAR* path, BYTE mode){
    EXPECT_EQ(path, "hello.motolog");
    EXPECT_EQ(mode, FA_WRITE | FA_CREATE_NEW);
    fopen_stub_called = true;
    return FR_OK;
  };
  stubs::f_close = [](FIL*){ return FR_OK; };

  Session sess(nullptr, 0, 0);
  EXPECT_TRUE(sess.open("hello.motolog"));

  EXPECT_TRUE(fopen_stub_called);
}

TEST(Session, open_failure)
{
  stubs::f_open = [](FIL*, const TCHAR* path, BYTE mode){
    return FR_DENIED;
  };
  stubs::f_close = [](FIL*){ return FR_OK; };

  Session sess(nullptr, 0, 0);
  EXPECT_FATAL_ERROR(sess.open("hello.motolog"));
}
