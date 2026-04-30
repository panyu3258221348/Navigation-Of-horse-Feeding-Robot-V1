#ifndef __CRC16_H
#define __CRC16_H
#include"config.h"
typedef struct
{
 unsigned char myadd;//本设备的地址
 unsigned char rcbuf[100]; //MODBUS接收缓冲区
 unsigned int timout;//MODbus的数据断续时间	
 unsigned char recount;//MODbus端口已经收到的数据个数
 unsigned char timrun;//MODbus定时器是否计时的标志
 unsigned char  reflag;//收到一帧数据的标志
 unsigned char Sendbuf[100]; //MODbus发送缓冲区	

}MODBUS;

extern MODBUS modbus;

/* CRC16计算函数，ptr-数据指针，len-数据长度，返回值-计算出的CRC16数值 */
uint crc16( uchar *puchMsg, uint usDataLen );

#endif
