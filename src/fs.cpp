#include "fs.hpp"
#include "hw.hpp"
#include "main.hpp"

#include <ff.h>
#include <ff_gen_drv.h>
#include <sd_diskio.h>
#include <fs.hpp>

static FATFS g_fat_fs;
static char  g_sd_path[4];

void fs::init()
{
    if (FATFS_LinkDriver(&SD_Driver, g_sd_path) != 0) {
        panic("Couldn't link FAT filesystem to SD card");
    } else if (/*BSP_SD_IsDetected() != SD_PRESENT*/false) {
        panic("SD card not detected");
    } else if (f_mount(&g_fat_fs, (TCHAR const*) g_sd_path, 1) != FR_OK) {
        panic("Couldn't mount FAT32 filesystem");
    }
}

void fs::deinit()
{
    (void) f_mount(nullptr, "", 0);
    FATFS_UnLinkDriver(g_sd_path);
}

extern "C" DWORD get_fattime(void)
{
    return static_cast<DWORD>(hw::get_rtc_timestamp());
}

extern "C" WCHAR ff_convert(WCHAR src, UINT)
{
    return src;
}

extern "C" WCHAR ff_wtoupper(WCHAR chr)
{
    return chr;
}
