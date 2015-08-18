

#define _CRT_SECURE_NO_DEPRECATE
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include <tchar.h>
#include <windows.h>
#include <stdio.h>
#include <deque>
#include <time.h>
#include <string.h>
#include <iostream>
#include "Serial.h"

#define HEADER_0 0
#define HEADER_1 1
#define HEADER_2 2
#define CONST_PACKET_DELIMITER_CHAR 0xAA
#define HEADER_XORED 0XA3
#define SENDER 3
#define PACKETTYPE 4
#define PACKETSIZE 5
#define CARD_ID_0 6
#define CARD_ID_1 7
#define CARD_ID_2 8
#define CONST_PACKETSIZE_MASTER 25
#define DATASTART 9
#define CONST_ONE_TO_ONE_SLEEP_TIME 130
#define MOTOR_CONTROL_CARD_1 3
#define SPEED_LIMIT 32767
#define RADIUS 0.2