//  Author: avishorp@gmail.com
//  Modded by: podor.ua@gmail.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include <TM1637Display_MOD.h>
#include <Arduino.h>

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A - 10
  0b01111100,    // b - 11
  0b00111001,    // C - 12
  0b01011000,    // c - 13
  0b01011110,    // d - 14
  0b01111001,    // E - 15
  0b01110001,    // F - 16
  0b00111101,    // G - 17
  0b01110110,    // H - 18
  0b01110100,    // h - 19
  0b00110000,    // I - 20
  0b00010000,    // i - 21
  0b00011110,    // J - 22
  0b00111000,    // L - 23
  0b00010101,    // M - 24
  0b01010100,    // n - 25
  0b00110111,    // N - 26
  0b00110111,    // o - 27
  0b01110011,    // P - 28
  0b01100111,    // q - 29
  0b01010000,    // r - 30
  0b00101101,    // S - 31
  0b01111000,    // t - 32
  0b00111110,    // U - 33
  0b00011100,    // v - 34
  0b00101010,    // W - 35
  0b01101110,    // y - 36
  0b00011011,    // Z - 37
  0b00000001,    // upper dash - 38
  0b01000000,    // middle dash - 39
  0b00001000,    // lower dash - 40
  0b01000001,    // upper and middle dash - 41
  0b01001000,    // middle and lower dash - 42
  0b00001001,    // upper and lower dash - 43
  0b01001001,    // all 3 dashes - 44
  };

TM1637Display::TM1637Display(uint8_t pinClk, uint8_t pinDIO)
{
	// Copy the pin numbers
	m_pinClk = pinClk;
	m_pinDIO = pinDIO;

	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
    pinMode(m_pinClk, INPUT);
    pinMode(m_pinDIO,INPUT);
	digitalWrite(m_pinClk, LOW);
	digitalWrite(m_pinDIO, LOW);
}

void TM1637Display::setBrightness(uint8_t brightness, bool on)
{
	m_brightness = (brightness & 0x7) | (on? 0x08 : 0x00);
}

void TM1637Display::setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    // Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();

	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

	// Write the data bytes
	for (uint8_t k=0; k < length; k++)
	  writeByte(segments[k]);

	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
}

void TM1637Display::showNumberDec(int num, bool leading_zero, uint8_t length, uint8_t pos)
{
  showNumberDecEx(num, 0, leading_zero, length, pos);
}

void TM1637Display::showNumberDecEx(int num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  uint8_t digits[4];
	const static int divisors[] = { 1, 10, 100, 1000 };
	bool leading = true;
	bool negative = false;
	
	if (num < 0) {
		num = num*-1;
		negative = true;
	}
	
	for(int8_t k = 0; k < 4; k++) {
	    int divisor = divisors[4 - 1 - k];
		int d = num / divisor;
		uint8_t digit = 0;

		if (d == 0) {
		  if ((leading_zero && k > 0) || !leading || (k == 3))
		      digit = encodeDigit(d);
	      else
			  if (negative)
				  digit = 0b01000000;
			  else
				digit = 0;
		}
		else {
			digit = encodeDigit(d);
			num -= d * divisor;
			leading = false;
		}
    
    // Add the decimal point/colon to the digit
    digit |= (dots & 0x80); 
    dots <<= 1;
    
    digits[k] = digit;
	}

	setSegments(digits + (4 - length), length, pos);
}


void TM1637Display::print(int chars[], int length)
{
	uint8_t data[] = {0x0, 0x0, 0x0, 0x0};

	for (uint8_t k=0; k < length; k++)
	  data[k] = (chars[k] < 0)?0x0:digitToSegment[chars[k]];

	return TM1637Display::setSegments(data);
}

void TM1637Display::printFloat(float num, int decimalCount, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
	int value;
	
	switch (decimalCount) {
        case 0: return ((int)num == num)?TM1637Display::showNumberDecEx(num, 0, 0, length, pos):TM1637Display::showNumberDecEx((int)num, dots, leading_zero, length, pos);
          break;
        case 1: value = (int)(num*10);
          break;
        case 2: value = (int)(num*100);
          break;
        case 3: value = (int)(num*1000);
          break;
    }

	return TM1637Display::showNumberDecEx(value, dots, leading_zero, length, pos);
}

void TM1637Display::bitDelay()
{
	delayMicroseconds(50);
}

void TM1637Display::start()
{
  pinMode(m_pinDIO, OUTPUT);
  bitDelay();
}

void TM1637Display::stop()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
	pinMode(m_pinClk, INPUT);
	bitDelay();
	pinMode(m_pinDIO, INPUT);
	bitDelay();
}

bool TM1637Display::writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(m_pinClk, OUTPUT);
    bitDelay();

	// Set data bit
    if (data & 0x01)
      pinMode(m_pinDIO, INPUT);
    else
      pinMode(m_pinDIO, OUTPUT);

    bitDelay();

	// CLK high
    pinMode(m_pinClk, INPUT);
    bitDelay();
    data = data >> 1;
  }

  // Wait for acknowledge
  // CLK to zero
  pinMode(m_pinClk, OUTPUT);
  pinMode(m_pinDIO, INPUT);
  bitDelay();

  // CLK to high
  pinMode(m_pinClk, INPUT);
  bitDelay();
  uint8_t ack = digitalRead(m_pinDIO);
  if (ack == 0)
    pinMode(m_pinDIO, OUTPUT);


  bitDelay();
  pinMode(m_pinClk, OUTPUT);
  bitDelay();

  return ack;
}

uint8_t TM1637Display::encodeDigit(uint8_t digit)
{
	return digitToSegment[digit];
}
