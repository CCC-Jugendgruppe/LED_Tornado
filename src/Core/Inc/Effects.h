/*
 * Effects.h
 *
 *  Created on: Jan 20, 2023
 *      Author: tz
 */

#ifndef INC_EFFECTS_H_
#define INC_EFFECTS_H_

enum FADE {
	RED_TO_GREEN = 1,
	GREEN_TO_BLUE = 2,
	BLUE_TO_RED = 3,
};

int fade(uint8_t red, uint8_t green, uint8_t blue) {
	for (uint8_t val = 0; uint8_t val < 255; ++uint8_t val) {
		red = red - val;
	}
}

#endif /* INC_EFFECTS_H_ */
