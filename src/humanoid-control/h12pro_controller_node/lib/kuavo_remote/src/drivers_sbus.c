/**
 * @file drivers_sbus.c
 * @author zhanglongbo (zhanglongbo@lejurobot.com)
 * @brief
 * @version 1.0
 * @date 2024-04-01
 * @copyright Copyright (c) 2024
 * @note
 */
#define _GNU_SOURCE

#include "drivers_sbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
//#include <termios.h>
#include <linux/termios.h>
//#include <sys/ioctl.h>
#include <linux/posix_types.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>


/* SBUS 超时计时器 */
uint8_t g_sbus_timeout;
/* SBUS 接收标志 */
uint8_t g_sbus_rec_flag;
/* SBUS 接收指示 */
uint8_t g_sbus_rec_index;

#ifdef UART_MODE
/* SBUS 接收缓存 */
uint8_t sbus_buf[35];
#endif

#ifdef SBUS_MODE
uint8_t sbus_buf[25];
#endif

/* SBUS 信息结构体 */
SbusInfoTypeDef SbusRxData;

/* SERIAL_PORT 信息结构体 */
struct termios tty;
int serial_port;

/**
 * @brief Ctrl + C
 * @note
 */
void handle_signal(int sig) {
    printf("Caught signal %d\n", sig);
    close(serial_port);
    printf("Seria port closed successfully due to Ctrl C\n");
    exit(0); // 正常退出程序
}

/**
 * @brief Initialize Sbus GPIO
 * @note
 */
int initSbus(void)
{
    serial_port = open(SERIAL_DEVICE_PATH,O_RDWR); //serial_port = open(SERIAL_DEVICE_PATH,O_RDWR | O_NOCTTY | O_NONBLOCK)
    if (serial_port <0)
    {
        perror("Error in opening serial port");
        return -1;
    }
    signal(SIGINT,handle_signal);
    signal(SIGALRM,timer_Hander);
    struct itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 50000;
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec =50000;
    setitimer(ITIMER_REAL,&timer,NULL);

#ifdef UART_MODE
    cfsetispeed(&tty,BAUD_RATE);
    cfsetospeed(&tty,BAUD_RATE);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
          
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_port,TCSANOW,&tty) !=0)
    {
        perror("Error in tcstattr");
        return -1;
    }
#endif

#ifdef SBUS_MODE
    struct termios2 tio = {};
     if (0 != ioctl(serial_port, TCGETS2, &tio)) {
         close(serial_port);
         serial_port= -1;
         return -1;
     }

     tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
              | IXON);
     tio.c_iflag |= (INPCK | IGNPAR);
     tio.c_oflag &= ~OPOST;
     tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
     tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
     /**
      * use BOTHER to specify speed directly in c_[io]speed member
      */
     tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
     tio.c_ispeed = 100000;
     tio.c_ospeed = 100000;
     tio.c_cc[VMIN]  = 0;
     tio.c_cc[VTIME] = 0;
     if (0 != ioctl(serial_port, TCSETS2, &tio)) {
         close(serial_port);
         serial_port= -1;
         return -1;
     }
     return 0;
#endif


    printf("Seria port opened successfully\n");
}

/**
 * @brief Initialize Sbus RX Data
 * @note
 */
static void initializeSbusRxData(void)
{
    SbusRxData.channel_1  = 1002;
    SbusRxData.channel_2  = 1002;
    SbusRxData.channel_3  = 1002;
    SbusRxData.channel_4  = 1002;
    SbusRxData.channel_5  = 282;
    SbusRxData.channel_6  = 1722;
    SbusRxData.channel_7  = 282;
    SbusRxData.channel_8  = 282;
    SbusRxData.channel_9  = 282;
    SbusRxData.channel_10 = 282;
    SbusRxData.channel_11 = 282;
    SbusRxData.channel_12 = 282;
    SbusRxData.channel_13 = 1002;
    SbusRxData.channel_14 = 1002;
    SbusRxData.channel_15 = 1002;
    SbusRxData.channel_16 = 1002;
    SbusRxData.sbus_state = 0;
}

/**
 * @brief  Receive SBUS Data
 * @note
 */
uint8_t recSbusData(void)
{
#ifdef UART_MODE
    uint8_t buf[35];
    int n = read(serial_port, buf, sizeof(buf));
    if (n > 0)
    {
#ifdef DEUG_SERIAL
        printf("Received %d bytes from serial port:\n", n);
        for (int i = 0; i < n; i++)
        {
            printf(" %02X", buf[i]);
        }
        printf("\n");
#endif
        if (buf[0] == 0x0F && g_sbus_rec_flag == 0 && n==32) /*判断SBUS帧头*/  /*The First Package*/
        {
            g_sbus_rec_flag = 1;
            for (int i = 0; i < n; i++)
            {
                sbus_buf[i] = buf[i];
                g_sbus_rec_index++;
            }
            return 0;
        }

        if (g_sbus_rec_flag == 1) /*The Secende Package*/
        {
            if (n==3){
                for (int i = 0; i < n; i++)
                {
                    sbus_buf[g_sbus_rec_index] = buf[i];
                    g_sbus_rec_index++;
                }
            }
            else{
                memset(sbus_buf, 0, 35);
                g_sbus_rec_index = 0;
                g_sbus_rec_flag = 0;
            }
        }

        if (g_sbus_rec_index == 35 && sbus_buf[g_sbus_rec_index - 2] == 0)   /*Parse Date Package*/
        {
#ifdef DEUG_SERIAL
            printf("SBUSReceive");
            for (int i = 0; i < g_sbus_rec_index; i++)
            {
                printf(" %02X", sbus_buf[i]);
            }
            printf("\n");
#endif
            parseSbusRxData();
            g_sbus_timeout = 0;
            memset(sbus_buf, 0, 35);
            g_sbus_rec_index = 0;
            g_sbus_rec_flag = 0;
        }
    }
    else if (n < 0)
    {
        perror("Error in reading from serial port");
        initializeSbusRxData();
        close(serial_port);
        return 1;
    }

    return 0;
#endif

#ifdef SBUS_MODE
    uint8_t buf[128];
    uint8_t buf_index =0;
    int n = read(serial_port, buf, sizeof(buf));
    if (n > 0)
    {
#ifdef DEUG_SERIAL
        printf("Received %d bytes from serial port:\n", n);
        for (int i = 0; i < n; i++)
        {
            printf(" %02X", buf[i]);
        }
        printf("\n");
#endif
        if (g_sbus_rec_flag == 0) /*判断SBUS帧头*/
        {
            // Check if 0x0F exists in the received data
            for (int i = 0; i < n; i++)
            {
                if (buf[i] == 0x0F)
                {
                    g_sbus_rec_flag = 1;  // Found SBUS frame header
                    buf_index = i; // Find Start index of SBUS frame
                    break;
                }
            }
        }

        if (g_sbus_rec_flag == 1)
        {
            // Append received data to sbus_buf starting from g_sbus_rec_index
            for (int i = buf_index; i < n; i++)
            {
                sbus_buf[g_sbus_rec_index++] = buf[i];
                if (g_sbus_rec_index >= 24 && sbus_buf[g_sbus_rec_index] == 0)
                {
#ifdef DEUG_SERIAL
                    printf("SBUSReceive");
                    for (int j = 0; j <= g_sbus_rec_index; j++)
                    {
                        printf(" %02X", sbus_buf[j]);
                    }
                    printf("\n");
#endif
                    parseSbusRxData();
                    g_sbus_timeout = 0;
                    memset(sbus_buf, 0, g_sbus_rec_index); // Clear sbus_buf
                    g_sbus_rec_index = 0; // Reset index
                    g_sbus_rec_flag = 0; // Reset flag
                    buf_index = i;
                    break; // Exit loop after processing one complete SBUS frame
                }
            }
            if (g_sbus_rec_flag == 0 && buf_index < n-1 && buf[buf_index+1]==0x0F) /**/
            {
                // Check if 0x0F exists in the received data
                for (int i = buf_index+1; i < n; i++)
                {
                    sbus_buf[g_sbus_rec_index++] = buf[i];
                    g_sbus_rec_flag = 1;
                }
            }
        }
    }
    else if (n < 0)
    {
        perror("Error in reading from serial port");
        initializeSbusRxData();
        close(serial_port);
        return 1;
    }

    return 0;
#endif
}

/**
 * @brief Parse SBUS Receive Data
 * @note
 */
void parseSbusRxData(void)
{

#ifdef UART_MODE
    if(sbus_buf[33] == 0x00) /* sbus_flag */
    {
        SbusRxData.sbus_state = 1;
        SbusRxData.channel_1  = (uint16_t)sbus_buf[1] << 8 | (uint16_t)sbus_buf[2];
        SbusRxData.channel_2  = (uint16_t)sbus_buf[3] << 8 | (uint16_t)sbus_buf[4];
        SbusRxData.channel_3  = (uint16_t)sbus_buf[5] << 8 | (uint16_t)sbus_buf[6];
        SbusRxData.channel_4  = (uint16_t)sbus_buf[7] << 8 | (uint16_t)sbus_buf[8];
        SbusRxData.channel_5  = (uint16_t)sbus_buf[9] << 8 | (uint16_t)sbus_buf[10];
        SbusRxData.channel_6  = (uint16_t)sbus_buf[11] << 8 | (uint16_t)sbus_buf[12];
        SbusRxData.channel_7  = (uint16_t)sbus_buf[13] << 8 | (uint16_t)sbus_buf[14];
        SbusRxData.channel_8  = (uint16_t)sbus_buf[15] << 8 | (uint16_t)sbus_buf[16];
        SbusRxData.channel_9  = (uint16_t)sbus_buf[17] << 8 | (uint16_t)sbus_buf[18];
        SbusRxData.channel_10 = (uint16_t)sbus_buf[19] << 8 | (uint16_t)sbus_buf[20];
        SbusRxData.channel_11 = (uint16_t)sbus_buf[21] << 8 | (uint16_t)sbus_buf[22];
        SbusRxData.channel_12 = (uint16_t)sbus_buf[23] << 8 | (uint16_t)sbus_buf[24];
        SbusRxData.channel_13 = (uint16_t)sbus_buf[25] << 8 | (uint16_t)sbus_buf[26];
        SbusRxData.channel_14 = (uint16_t)sbus_buf[27] << 8 | (uint16_t)sbus_buf[28];
        SbusRxData.channel_15 = (uint16_t)sbus_buf[29] << 8 | (uint16_t)sbus_buf[30];
        SbusRxData.channel_16 = (uint16_t)sbus_buf[31] << 8 | (uint16_t)sbus_buf[32];
    }
    else
    {
        SbusRxData.sbus_state = 0;
        initializeSbusRxData( );
    }

#endif

#ifdef SBUS_MODE
    if (sbus_buf[23] == 0x00) /* sbus_flag */
    {
        SbusRxData.sbus_state = 1;
        SbusRxData.channel_1  = ((uint16_t)sbus_buf[1] >> 0 | ((uint16_t)sbus_buf[2] << 8)) & 0x07FF;
        SbusRxData.channel_2  = ((uint16_t)sbus_buf[2] >> 3 | ((uint16_t)sbus_buf[3] << 5)) & 0x07FF;
        SbusRxData.channel_3  = ((uint16_t)sbus_buf[3] >> 6 | ((uint16_t)sbus_buf[4] << 2) | (uint16_t)sbus_buf[5] << 10) & 0x07FF;
        SbusRxData.channel_4  = ((uint16_t)sbus_buf[5] >> 1 | ((uint16_t)sbus_buf[6] << 7)) & 0x07FF;
        SbusRxData.channel_5  = ((uint16_t)sbus_buf[6] >> 4 | ((uint16_t)sbus_buf[7] << 4)) & 0x07FF;
        SbusRxData.channel_6  = ((uint16_t)sbus_buf[7] >> 7 | ((uint16_t)sbus_buf[8] << 1) | (uint16_t)sbus_buf[9] << 9) & 0x07FF;
        SbusRxData.channel_7  = ((uint16_t)sbus_buf[9] >> 2 | ((uint16_t)sbus_buf[10] << 6)) & 0x07FF;
        SbusRxData.channel_8  = ((uint16_t)sbus_buf[10] >> 5 | ((uint16_t)sbus_buf[11] << 3)) & 0x07FF;
        SbusRxData.channel_9  = ((uint16_t)sbus_buf[12] << 0 | ((uint16_t)sbus_buf[13] << 8)) & 0x07FF;
        SbusRxData.channel_10 = ((uint16_t)sbus_buf[13] >> 3 | ((uint16_t)sbus_buf[14] << 5)) & 0x07FF;
        SbusRxData.channel_11 = ((uint16_t)sbus_buf[14] >> 6 | ((uint16_t)sbus_buf[15] << 2) | (uint16_t)sbus_buf[16] << 10) & 0x07FF;
        SbusRxData.channel_12 = ((uint16_t)sbus_buf[16] >> 1 | ((uint16_t)sbus_buf[17] << 7)) & 0x07FF;
        SbusRxData.channel_13 = ((uint16_t)sbus_buf[17] >> 4 | ((uint16_t)sbus_buf[18] << 4)) & 0x07FF;
        SbusRxData.channel_14 = ((uint16_t)sbus_buf[18] >> 7 | ((uint16_t)sbus_buf[19] << 1) | (uint16_t)sbus_buf[20] << 9) & 0x07FF;
        SbusRxData.channel_15 = ((uint16_t)sbus_buf[20] >> 2 | ((uint16_t)sbus_buf[21] << 6)) & 0x07FF;
        SbusRxData.channel_16 = ((uint16_t)sbus_buf[21] >> 5 | ((uint16_t)sbus_buf[22] << 3)) & 0x07FF;
    }
    else
    {
        SbusRxData.sbus_state = 0;
        initializeSbusRxData( );
    }
#endif
}

/**
 * @brief Print SBUS Info
 * @note
 */
void printSbusInfo(SbusInfoTypeDef sbus_info)
{
   printf("---------------------------------------------------------\n");
   printf("Channel 1: %u\n",sbus_info.channel_1);
   printf("Channel 2: %u\n",sbus_info.channel_2);
   printf("Channel 3: %u\n",sbus_info.channel_3);
   printf("Channel 4: %u\n",sbus_info.channel_4);
   printf("Channel 5: %u\n",sbus_info.channel_5);
   printf("Channel 6: %u\n",sbus_info.channel_6);
   printf("Channel 7: %u\n",sbus_info.channel_7);
   printf("Channel 8: %u\n",sbus_info.channel_8);
   printf("Channel 9: %u\n",sbus_info.channel_9);
   printf("Channel 10: %u\n",sbus_info.channel_10);
   printf("Channel 11: %u\n",sbus_info.channel_11);
   printf("Channel 12: %u\n",sbus_info.channel_12);
   printf("Channel 13: %u\n",sbus_info.channel_13);
   printf("Channel 14: %u\n",sbus_info.channel_14);
   printf("Channel 15: %u\n",sbus_info.channel_15);
   printf("Channel 16: %u\n",sbus_info.channel_16);
   printf("SBUS State: %u\n",sbus_info.sbus_state);
   printf("---------------------------------------------------------\n");
}

/**
 * @brief Check SBUS Time Out
 * @note
 */
void checkSbusTimeOut(void)
{
    if (g_sbus_timeout >= 2)
    {
        initializeSbusRxData( );
    }
}

/**
 * @brief Timer
 * @note
 */
void timer_Hander(int signum)
{
    g_sbus_timeout++;
#ifdef DEUG_CHANNEL
    printSbusInfo(SbusRxData);
#endif
}
