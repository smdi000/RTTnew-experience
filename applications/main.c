#include <rtthread.h>

int main(void)
{
    rt_kprintf("===== FW MIN SAFE TEST 999 =====\r\n");
    rt_kprintf("main start\r\n");

    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return 0;
}
