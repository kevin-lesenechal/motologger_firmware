#include <sys/stat.h>
#include <cerrno>
#include <sys/unistd.h>

#include <hw.hpp>

extern "C" {

int _close(int)
{
    return -1;
}

int _fstat(int, struct stat* st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int)
{
    return 1;
}

int _lseek(int, int, int)
{
    return 0;
}

int _open(const char*, int, int)
{
    return -1;
}

int _read(int, char*, int)
{
    return -1;
}

int _write(int file, char* data, int len)
{
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
        errno = EBADF;
        return -1;
    }
    HAL_StatusTypeDef status =
                          HAL_UART_Transmit(&hw::dev::uart, (uint8_t*) data, (uint16_t) len, 1000);
    return status == HAL_OK ? len : 0;
}

caddr_t _sbrk(int incr)
{
    extern char _end; // Defined by the linker.
    //extern char _Heap_Limit; // Defined by the linker.

    static char* current_heap_end;
    char       * current_block_address;

    // first allocation
    if (current_heap_end == 0)
        current_heap_end  = &_end;
    current_block_address = current_heap_end;
    // increment and align to 4-octet border
    incr                  = (incr + 3) & (~3);
    current_heap_end += incr;
    // Overflow?
    /*if (current_heap_end > &_Heap_Limit)
    {
        errno = ENOMEM;
        current_heap_end = current_block_address;
        return (caddr_t)-1;
    }*/
    return (caddr_t) current_block_address;
}

int _getpid()
{
    return 1;
}

void _kill(int)
{}

void _exit(int)
{
    while (1);
}

} // extern "C"
