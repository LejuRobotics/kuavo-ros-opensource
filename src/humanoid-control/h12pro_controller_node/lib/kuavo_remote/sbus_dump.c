/**
 * @file sbus_dump.c
 * @brief 独立 SBUS 底层数据打印工具（不依赖 ROS / msg）
 *
 * 直接链接 drivers_sbus.c，循环接收并打印全部 16 通道原始解码值，
 * 用于验证遥控器(G11)通过 USB 接收器实际输出的通道值与值域。
 *
 * 编译: 见 run_sbus_dump.sh
 * 运行: sudo ./sbus_dump          (需 root 访问 /dev/usb_remote)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "drivers_sbus.h"

/* 打印开关: 置 1 时每 N 帧打印全部通道; 置 0 时只打印发生变化的行(找哪个通道在动)。
 * 用 #ifndef 包住, 使 `gcc -DPRINT_ALL_ALWAYS=1`(run_sbus_dump.sh --print-all) 能生效;
 * 之前是无条件 #define, 命令行 -D 会被这里覆盖(仅告警), --print-all 形同虚设。 */
#ifndef PRINT_ALL_ALWAYS
#define PRINT_ALL_ALWAYS   0
#endif
/* 打印间隔(帧): 每 N 次 recSbusData 打印一次, 避免刷屏 */
#define PRINT_EVERY_N_FRAME 3

static SbusInfoTypeDef s_last = {0};
static int s_printed_once = 0;

static void dump_line(const char *tag, const SbusInfoTypeDef *s)
{
    printf("[%s] %4u %4u %4u %4u | %4u %4u %4u %4u | %4u %4u %4u %4u | %4u %4u %4u %4u | st=%u\n",
           tag,
           s->channel_1,  s->channel_2,  s->channel_3,  s->channel_4,
           s->channel_5,  s->channel_6,  s->channel_7,  s->channel_8,
           s->channel_9,  s->channel_10, s->channel_11, s->channel_12,
           s->channel_13, s->channel_14, s->channel_15, s->channel_16,
           s->sbus_state);
    fflush(stdout);
}

int main(void)
{
    printf("=== SBUS bottom-layer dump (no ROS) ===\n");
    printf("device: %s | SBUS_MODE(11bit, 100000 baud)\n\n", SERIAL_DEVICE_PATH);
    printf("frame:  ch1..ch4 | ch5..ch8 | ch9..ch12 | ch13..ch16 | state\n");
    printf("------------------------------------------------------------\n");

    if (initSbus() != 0) {
        fprintf(stderr, "initSbus failed (is receiver plugged? /dev/usb_remote exist? run as root?)\n");
        return -1;
    }

    int frame_count = 0;
    while (1) {
        if (recSbusData() == 1) {
            break; /* 串口错误 */
        }
        frame_count++;

#if PRINT_ALL_ALWAYS
        if (frame_count % PRINT_EVERY_N_FRAME == 0) {
            dump_line("all", &SbusRxData);
        }
#else
        /* 只打印变化的行, 便于一眼看出哪个通道在动 */
        if (memcmp(&SbusRxData, &s_last, sizeof(SbusRxData)) != 0) {
            s_last = SbusRxData;
            dump_line("chg", &SbusRxData);
        } else if (!s_printed_once && SbusRxData.sbus_state == 1) {
            s_printed_once = 1;
            dump_line("init", &SbusRxData);
        }
#endif

        checkSbusTimeOut();
        usleep(2000); /* 2ms, 让出 CPU */
    }

    printf("serial error, exit\n");
    return -1;
}
