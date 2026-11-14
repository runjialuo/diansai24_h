#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

#define KEY_CNT     0x01U
#define KEY_START   0x02U
#define KEY_RESET   0x40U

void Key_Init(void);
void Key_Scan(void);
uint8_t Key_GetEvent(void);

#endif
