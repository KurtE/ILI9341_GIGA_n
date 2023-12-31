#ifndef BASE_TRANSITION_H__
#define BASE_TRANSITION_H__

#include <Arduino.h>
#include <ILI9341_GIGA_n.h>
#include "MathUtil.h"

class BaseTransition {
public:
	BaseTransition(){};

	virtual void init( ILI9341_GIGA_n tft );
	virtual void restart( ILI9341_GIGA_n tft, uint_fast16_t color );
	virtual void perFrame( ILI9341_GIGA_n tft, FrameParams frameParams );
	virtual boolean isComplete();
};

void BaseTransition::init( ILI9341_GIGA_n tft ) {
	// Extend me
}

void BaseTransition::restart( ILI9341_GIGA_n tft, uint_fast16_t color ) {
	// Extend me
}

void BaseTransition::perFrame( ILI9341_GIGA_n tft, FrameParams frameParams ) {
	// Extend me
}

boolean BaseTransition::isComplete() {
	// Extend me
	return false;
}

#endif
