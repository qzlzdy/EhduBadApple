/*
 * Displayer.h
 *
 *  Created on: Mar 22, 2022
 *      Author: qzlzdy
 */

#ifndef INC_DISPLAYER_H_
#define INC_DISPLAYER_H_

#include "Displayer.h"

#include "main.h"
#include <cstdint>

namespace ehdu {

class Displayer{
public:
	static Displayer *getInstance();
	Displayer(const Displayer &) = delete;
	Displayer &operator=(const Displayer &) = delete;
	void drawLine(uint16_t line, uint8_t *pixels);
	void fill(uint16_t bx, uint16_t by, uint16_t ex,
              uint16_t ey, uint16_t color);
private:
	Displayer();
	void setCursor(uint16_t x, uint16_t y);
	void writeReg(uint8_t reg);
	void writeData(uint16_t data);
	static Displayer *displayer;
};

} /* namespace ehdu */

#endif /* INC_DISPLAYER_H_ */
