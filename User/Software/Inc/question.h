#ifndef __QUESTION_H
#define __QUESTION_H

#include <stdint.h>

/*==============================================================
 *  四个问题统一接口
 *
 *  每个问题两个函数，结构完全一致：
 *      QuesN_Start()    : 进入该题时调用一次（按键里触发）
 *      QuesN_Task10ms() : 每 10ms 调用
 *                         返回 0 = 进行中
 *                              1 = 完成
 *                              2 = 超时 / 故障
 *==============================================================*/

void    Ques1_Start(void);
uint8_t Ques1_Task10ms(void);

void    Ques2_Start(void);
uint8_t Ques2_Task10ms(void);

void    Ques3_Start(void);
uint8_t Ques3_Task10ms(void);

void    Ques4_Start(void);
uint8_t Ques4_Task10ms(void);


#endif /* __QUESTION_H */
