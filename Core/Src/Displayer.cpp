/*
 * Displayer.cpp
 *
 *  Created on: Mar 22, 2022
 *      Author: qzlzdy
 */

#include "Displayer.h"

#include "main.h"

using namespace std;

namespace ehdu {

Displayer *Displayer::getInstance(){
	if(displayer == nullptr){
		displayer = new Displayer;
	}
	return displayer;
}

void Displayer::drawLine(uint16_t line, uint8_t *pixels){
	setCursor(0, line);
	writeReg(0x2C);
	for(int i = 0; i < 240; ++i){
		writeData((pixels[i * 2] << 8) | pixels[i * 2 + 1]);
	}
}

void Displayer::fill(uint16_t bx, uint16_t by, uint16_t ex,
        uint16_t ey, uint16_t color){
	for(int i = by; i <= ey; ++i){
		setCursor(bx, i);
		writeReg(0x2C);
		for(int j = bx; j <= ex; ++j){
			writeData(color);
		}
	}
}

Displayer::Displayer() {
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);
	writeReg(0xCF);
	writeData(0x00);
	writeData(0xC1);
	writeData(0x30);
	writeReg(0xED);
	writeData(0x64);
	writeData(0x03);
	writeData(0X12);
	writeData(0X81);
	writeReg(0xE8);
	writeData(0x85);
	writeData(0x10);
	writeData(0x7A);
	writeReg(0xCB);
	writeData(0x39);
	writeData(0x2C);
	writeData(0x00);
	writeData(0x34);
	writeData(0x02);
	writeReg(0xF7);
	writeData(0x20);
	writeReg(0xEA);
	writeData(0x00);
	writeData(0x00);
	writeReg(0xC0);
	writeData(0x1B);
	writeReg(0xC1);
	writeData(0x01);
	writeReg(0xC5);
	writeData(0x30);
	writeData(0x30);
	writeReg(0xC7);
	writeData(0XB7);
	writeReg(0x36);
	writeData(0x48);
	writeReg(0x3A);
	writeData(0x55);
	writeReg(0xB1);
	writeData(0x00);
	writeData(0x1A);
	writeReg(0xB6);
	writeData(0x0A);
	writeData(0xA2);
	writeReg(0xF2);
	writeData(0x00);
	writeReg(0x26);
	writeData(0x01);
	writeReg(0xE0);
	writeData(0x0F);
	writeData(0x2A);
	writeData(0x28);
	writeData(0x08);
	writeData(0x0E);
	writeData(0x08);
	writeData(0x54);
	writeData(0XA9);
	writeData(0x43);
	writeData(0x0A);
	writeData(0x0F);
	writeData(0x00);
	writeData(0x00);
	writeData(0x00);
	writeData(0x00);
	writeReg(0XE1);
	writeData(0x00);
	writeData(0x15);
	writeData(0x17);
	writeData(0x07);
	writeData(0x11);
	writeData(0x06);
	writeData(0x2B);
	writeData(0x56);
	writeData(0x3C);
	writeData(0x05);
	writeData(0x10);
	writeData(0x0F);
	writeData(0x3F);
	writeData(0x3F);
	writeData(0x0F);
	writeReg(0x2B);
	writeData(0x00);
	writeData(0x00);
	writeData(0x01);
	writeData(0x3f);
	writeReg(0x2A);
	writeData(0x00);
	writeData(0x00);
	writeData(0x00);
	writeData(0xef);
	writeReg(0x11);
	HAL_Delay(100);
	writeReg(0x29);

	fill(0, 0, 239, 319, 0xFFFF);
}

void Displayer::setCursor(uint16_t x, uint16_t y){
	writeReg(0x2A);
	writeData(x >> 8);
	writeData(x & 0XFF);
	writeReg(0x2B);
	writeData(y >> 8);
	writeData(y & 0XFF);
}

void Displayer::writeReg(uint8_t reg){
	*(__IO uint16_t *)(0x60000000) = reg;
}

void Displayer::writeData(uint16_t data){
	*(__IO uint16_t *)(0x60020000) = data;
}

Displayer *Displayer::displayer = nullptr;

} /* namespace ehdu */
