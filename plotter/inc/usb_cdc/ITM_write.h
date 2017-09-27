/*
 * ITM_write.h
 *
 *  Created on: 5.9.2016
 *      Author: krl
 */

#ifndef ITM_WRITE_H_
#define ITM_WRITE_H_

#ifdef __cplusplus
extern "C" {
#endif

void ITM_init(void);
int ITM_write(const char *pcBuffer);

#ifdef __cplusplus
}
#endif

#endif /* ITM_WRITE_H_ */
