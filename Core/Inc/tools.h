/*
 * tools.h
 *
 *  Created on: Mar 7, 2024
 *      Author: SoftwareEngineer_01
 */

#ifndef SRC_TOOLS_H_
#define SRC_TOOLS_H_

uint32_t msTimer_Now();
uint32_t msTimer_DiffFrom(uint32_t last_ms);

unsigned short Crc16(unsigned char *pcBlock, unsigned short len);
unsigned char Crc8(unsigned char *pcBlock, unsigned int len);

#endif /* SRC_TOOLS_H_ */
