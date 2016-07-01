#include "TFT_22_ILI9225.h"

// Constructor when using software SPI.  All output pins are configurable.
TFT_22_ILI9225::TFT_22_ILI9225(uint8_t rst, uint8_t rs, uint8_t cs, uint8_t sdi, uint8_t clk, uint8_t led) {
	_rst  = rst;
	_rs   = rs;
	_cs   = cs;
	_sdi  = sdi;
	_clk  = clk;
	_led  = led;
	hwSPI = false;
}

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
TFT_22_ILI9225::TFT_22_ILI9225(uint8_t rst, uint8_t rs, uint8_t cs, uint8_t led) {
	_rst  = rst;
	_rs   = rs;
	_cs   = cs;
	_sdi  = _clk = 0;
	_led  = led;
	hwSPI = true;
}


void TFT_22_ILI9225::_orientCoordinates(uint16_t &x1, uint16_t &y1) {

	switch (_orientation) {
	case 0:  // ok
		break;
	case 1: // ok
		y1 = _maxY - y1 - 1;
		_swap(x1, y1);
		break;
	case 2: // ok
		x1 = _maxX - x1 - 1;
		y1 = _maxY - y1 - 1;
		break;
	case 3: // ok
		x1 = _maxX - x1 - 1;
		_swap(x1, y1);
		break;
	}
}


void TFT_22_ILI9225::_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	_orientCoordinates(x0, y0);
	_orientCoordinates(x1, y1);

	if (x1<x0) _swap(x0, x1);
	if (y1<y0) _swap(y0, y1);

	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1,x1);
	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2,x0);

	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1,y1);
	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2,y0);

	_writeRegister(ILI9225_RAM_ADDR_SET1,x0);
	_writeRegister(ILI9225_RAM_ADDR_SET2,y0);

	_writeCommand(0x00, 0x22);
}


void TFT_22_ILI9225::begin() {

	// Set up pins
	pinMode(_rs, OUTPUT);
	pinMode(_cs, OUTPUT);
	pinMode(_rst, OUTPUT);
	if (_led) pinMode(_led, OUTPUT);

	if (hwSPI) { // Using hardware SPI
		SPI.begin();
		SPI.setClockDivider(SPI_CLOCK_DIV2); // 4 MHz (half speed)
		//SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
		SPI.setBitOrder(MSBFIRST);
		SPI.setDataMode(SPI_MODE0);
	} else {
		pinMode(_clk, OUTPUT);
		pinMode(_sdi, OUTPUT);
	}

	// Turn on backlight
	if (_led) digitalWrite(_led, HIGH);

	// Initialization Code
	digitalWrite(_rst, 1); // Pull the reset pin high to release the ILI9225C from the reset status
	delay(1); 
	digitalWrite(_rst, 0); // Pull the reset pin low to reset ILI9225
	delay(10);
	digitalWrite(_rst, 1); // Pull the reset pin high to release the ILI9225C from the reset status
	delay(50);

	/* Start Initial Sequence */
	/* Set SS bit and direction output from S528 to S1 */
	_writeRegister(ILI9225_POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB
	_writeRegister(ILI9225_POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
	_writeRegister(ILI9225_POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
	_writeRegister(ILI9225_POWER_CTRL4, 0x0000); // Set GVDD
	_writeRegister(ILI9225_POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
	delay(40); 

	// Power-on sequence
	_writeRegister(ILI9225_POWER_CTRL2, 0x0018); // Set APON,PON,AON,VCI1EN,VC
	_writeRegister(ILI9225_POWER_CTRL3, 0x6121); // Set BT,DC1,DC2,DC3
	_writeRegister(ILI9225_POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
	_writeRegister(ILI9225_POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
	_writeRegister(ILI9225_POWER_CTRL1, 0x0800); // Set SAP,DSTB,STB
	delay(10);
	_writeRegister(ILI9225_POWER_CTRL2, 0x103B); // Set APON,PON,AON,VCI1EN,VC
	delay(50);

	_writeRegister(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
	_writeRegister(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
	_writeRegister(ILI9225_ENTRY_MODE, 0x1030); // set GRAM write direction and BGR=1.
	_writeRegister(ILI9225_DISP_CTRL1, 0x0000); // Display off
	_writeRegister(ILI9225_BLANK_PERIOD_CTRL1, 0x0808); // set the back porch and front porch
	_writeRegister(ILI9225_FRAME_CYCLE_CTRL, 0x1100); // set the clocks number per line
	_writeRegister(ILI9225_INTERFACE_CTRL, 0x0000); // CPU interface
	_writeRegister(ILI9225_OSC_CTRL, 0x0D01); // Set Osc  /*0e01*/
	_writeRegister(ILI9225_VCI_RECYCLING, 0x0020); // Set VCI recycling
	_writeRegister(ILI9225_RAM_ADDR_SET1, 0x0000); // RAM Address
	_writeRegister(ILI9225_RAM_ADDR_SET2, 0x0000); // RAM Address

	/* Set GRAM area */
	_writeRegister(ILI9225_GATE_SCAN_CTRL, 0x0000); 
	_writeRegister(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB); 
	_writeRegister(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000); 
	_writeRegister(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000); 
	_writeRegister(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB); 
	_writeRegister(ILI9225_PARTIAL_DRIVING_POS2, 0x0000); 
	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF); 
	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000); 
	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB); 
	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000); 

	/* Set GAMMA curve */
	_writeRegister(ILI9225_GAMMA_CTRL1, 0x0000); 
	_writeRegister(ILI9225_GAMMA_CTRL2, 0x0808); 
	_writeRegister(ILI9225_GAMMA_CTRL3, 0x080A); 
	_writeRegister(ILI9225_GAMMA_CTRL4, 0x000A); 
	_writeRegister(ILI9225_GAMMA_CTRL5, 0x0A08); 
	_writeRegister(ILI9225_GAMMA_CTRL6, 0x0808); 
	_writeRegister(ILI9225_GAMMA_CTRL7, 0x0000); 
	_writeRegister(ILI9225_GAMMA_CTRL8, 0x0A00); 
	_writeRegister(ILI9225_GAMMA_CTRL9, 0x0710); 
	_writeRegister(ILI9225_GAMMA_CTRL10, 0x0710); 

	_writeRegister(ILI9225_DISP_CTRL1, 0x0012); 
	delay(50); 
	_writeRegister(ILI9225_DISP_CTRL1, 0x1017);

	setBacklight(true);
	setOrientation(0);

	// Initialize variables
	setBackgroundColor( COLOR_BLACK );

	clear();
}


void TFT_22_ILI9225::clear() {
	uint8_t old = _orientation;
	setOrientation(0);
	fillRectangle(0, 0, _maxX - 1, _maxY - 1, COLOR_BLACK);
	setOrientation(old);
	delay(10);
}


void TFT_22_ILI9225::invert(boolean flag) {
	_writeCommand(0x00, flag ? ILI9225C_INVON : ILI9225C_INVOFF);
}


void TFT_22_ILI9225::setBacklight(boolean flag) {
	if (_led) digitalWrite(_led, flag);
}


void TFT_22_ILI9225::setDisplay(boolean flag) {
	if (flag) {
		_writeRegister(0x00ff, 0x0000);
		_writeRegister(ILI9225_POWER_CTRL1, 0x0000);
		delay(50);
		_writeRegister(ILI9225_DISP_CTRL1, 0x1017);
		delay(200);
	} else {
		_writeRegister(0x00ff, 0x0000);
		_writeRegister(ILI9225_DISP_CTRL1, 0x0000);
		delay(50);
		_writeRegister(ILI9225_POWER_CTRL1, 0x0003);
		delay(200);
	}
}


void TFT_22_ILI9225::setOrientation(uint8_t orientation) {

	_orientation = orientation % 4;

	switch (_orientation) {
	case 0:
		_maxX = ILI9225_LCD_WIDTH;
		_maxY = ILI9225_LCD_HEIGHT;
		break;
	case 1:
		_maxX = ILI9225_LCD_HEIGHT;
		_maxY = ILI9225_LCD_WIDTH;
		break;
	case 2:
		_maxX = ILI9225_LCD_WIDTH;
		_maxY = ILI9225_LCD_HEIGHT;
		break;
	case 3:
		_maxX = ILI9225_LCD_HEIGHT;
		_maxY = ILI9225_LCD_WIDTH;
		break;
	}
}


uint8_t TFT_22_ILI9225::getOrientation() {
	return _orientation;
}


void TFT_22_ILI9225::drawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	drawLine(x1, y1, x1, y2, color);
	drawLine(x1, y1, x2, y1, color);
	drawLine(x1, y2, x2, y2, color);
	drawLine(x2, y1, x2, y2, color);
}


void TFT_22_ILI9225::fillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {

	_setWindow(x1, y1, x2, y2);

	for(uint16_t t=(y2 - y1 + 1) * (x2 - x1 + 1); t > 0; t--)
		_writeData(color >> 8, color);
}


void TFT_22_ILI9225::drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {

	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	drawPixel(x0, y0 + r, color);
	drawPixel(x0, y0-  r, color);
	drawPixel(x0 + r, y0, color);
	drawPixel(x0 - r, y0, color);

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		drawPixel(x0 + x, y0 + y, color);
		drawPixel(x0 - x, y0 + y, color);
		drawPixel(x0 + x, y0 - y, color);
		drawPixel(x0 - x, y0 - y, color);
		drawPixel(x0 + y, y0 + x, color);
		drawPixel(x0 - y, y0 + x, color);
		drawPixel(x0 + y, y0 - x, color);
		drawPixel(x0 - y, y0 - x, color);
	}
}


void TFT_22_ILI9225::fillCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color) {

	int16_t f = 1 - radius;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * radius;
	int16_t x = 0;
	int16_t y = radius;

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		drawLine(x0 + x, y0 + y, x0 - x, y0 + y, color); // bottom
		drawLine(x0 + x, y0 - y, x0 - x, y0 - y, color); // top
		drawLine(x0 + y, y0 - x, x0 + y, y0 + x, color); // right
		drawLine(x0 - y, y0 - x, x0 - y, y0 + x, color); // left
	}
	fillRectangle(x0-x, y0-y, x0+x, y0+y, color);
}


void TFT_22_ILI9225::drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {

	// Classic Bresenham algorithm
	int16_t steep = abs(y2 - y1) > abs(x2 - x1);
	int16_t dx, dy;

	if (steep) {
		_swap(x1, y1);
		_swap(x2, y2);
	}

	if (x1 > x2) {
		_swap(x1, x2);
		_swap(y1, y2);
	}

	dx = x2 - x1;
	dy = abs(y2 - y1);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y1 < y2) ystep = 1;
	else ystep = -1;


	for (; x1<=x2; x1++) {
		if (steep) drawPixel(y1, x1, color);
		else       drawPixel(x1, y1, color);

		err -= dy;
		if (err < 0) {
			y1 += ystep;
			err += dx;
		}
	}
}


void TFT_22_ILI9225::drawPixel(uint16_t x1, uint16_t y1, uint16_t color) {

	if((x1 < 0) || (x1 >= _maxX) || (y1 < 0) || (y1 >= _maxY)) return;

	_setWindow(x1, y1, x1+1, y1+1);
	_orientCoordinates(x1, y1);
	_writeData(color >> 8, color);
}


uint16_t TFT_22_ILI9225::maxX() {
	return _maxX;
}


uint16_t TFT_22_ILI9225::maxY() {
	return _maxY;
}


uint16_t TFT_22_ILI9225::setColor(uint8_t red8, uint8_t green8, uint8_t blue8) {
	// rgb16 = red5 green6 blue5
	return (red8 >> 3) << 11 | (green8 >> 2) << 5 | (blue8 >> 3);
}


void TFT_22_ILI9225::splitColor(uint16_t rgb, uint8_t &red, uint8_t &green, uint8_t &blue) {
	// rgb16 = red5 green6 blue5
	red   = (rgb & 0b1111100000000000) >> 11 << 3;
	green = (rgb & 0b0000011111100000) >>  5 << 2;
	blue  = (rgb & 0b0000000000011111)       << 3;
}


void TFT_22_ILI9225::_swap(uint16_t &a, uint16_t &b) {
	uint16_t w = a;
	a = b;
	b = w;
}

// Utilities
void TFT_22_ILI9225::_writeCommand(uint8_t HI, uint8_t LO) {
	digitalWrite(_rs, LOW);
	digitalWrite(_cs, LOW);
	SPI.transfer(HI);
	SPI.transfer(LO);
	digitalWrite(_cs, HIGH);
}


void TFT_22_ILI9225::_writeData(uint8_t HI, uint8_t LO) {
	digitalWrite(_rs, HIGH);
	digitalWrite(_cs, LOW);
	SPI.transfer(HI);
	SPI.transfer(LO);
	digitalWrite(_cs, HIGH);
}


void TFT_22_ILI9225::_writeRegister(uint16_t reg, uint16_t data) {
	_writeCommand(reg >> 8, reg & 255);
	_writeData(data >> 8, data & 255);
}


void TFT_22_ILI9225::drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color) {
	drawLine(x1, y1, x2, y2, color);
	drawLine(x2, y2, x3, y3, color);
	drawLine(x3, y3, x1, y1, color);
}


void TFT_22_ILI9225::fillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color) {

	uint16_t a, b, y, last;

	// Sort coordinates by Y order (y3 >= y2 >= y1)
	if (y1 > y2) {
		_swap(y1, y2); _swap(x1, x2);
	}
	if (y2 > y3) {
		_swap(y3, y2); _swap(x3, x2);
	}
	if (y1 > y2) {
		_swap(y1, y2); _swap(x1, x2);
	}

	if (y1 == y3) { // Handle awkward all-on-same-line case as its own thing
		a = b = x1;
		if (x2 < a)      a = x2;
		else if (x2 > b) b = x2;
		if (x3 < a)      a = x3;
		else if (x3 > b) b = x3;
			drawLine(a, y1, b, y1, color);
		return;
	}

	uint16_t	dx11 = x2 - x1,
				dy11 = y2 - y1,
				dx12 = x3 - x1,
				dy12 = y3 - y1,
				dx22 = x3 - x2,
				dy22 = y3 - y2,
				sa   = 0,
				sb   = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y2=y3 (flat-bottomed triangle), the scanline y2
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y2 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y1=y2
	// (flat-topped triangle).
	if (y2 == y3) last = y2;   // Include y2 scanline
	else          last = y2 - 1; // Skip it

	for (y = y1; y <= last; y++) {
	a   = x1 + sa / dy11;
	b   = x1 + sb / dy12;
	sa += dx11;
	sb += dx12;
	/* longhand:
	a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
	b = x1 + (x3 - x1) * (y - y1) / (y3 - y1);
	*/
	if (a > b) _swap(a,b);
		drawLine(a, y, b, y, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y2=y3.
	sa = dx22 * (y - y2);
	sb = dx12 * (y - y1);
	for (; y<=y3; y++) {
		a   = x2 + sa / dy22;
		b   = x1 + sb / dy12;
		sa += dx22;
		sb += dx12;
		/* longhand:
		a = x2 + (x3 - x2) * (y - y2) / (y3 - y2);
		b = x1 + (x3 - x1) * (y - y1) / (y3 - y1);
		*/
		if (a > b) _swap(a,b);
			drawLine(a, y, b, y, color);
	}
}


void TFT_22_ILI9225::setBackgroundColor(uint16_t color) {
	_bgColor = color;
}


void TFT_22_ILI9225::setFont(uint8_t* font) {

	cfont.font 	   = font;
	cfont.width    = readFontByte(0);
	cfont.height   = readFontByte(1);
	cfont.offset   = readFontByte(2);
	cfont.numchars = readFontByte(3);
	cfont.nbrows   = cfont.height / 8;

	if (cfont.height % 8) cfont.nbrows++;  // Set number of bytes used by height of font in multiples of 8
}


void TFT_22_ILI9225::drawText(uint16_t x, uint16_t y, String s, uint16_t color) {

	uint16_t currx = x;

	// Print every character in string
	for (uint8_t k = 0; k < s.length(); k++) {
		currx += drawChar(currx, y, s.charAt(k), color) + 1;
	}
}


uint16_t TFT_22_ILI9225::drawChar(uint16_t x, uint16_t y, uint16_t ch, uint16_t color) {

	uint8_t charData, charWidth;
	uint8_t h, i, j, k;
	uint16_t charOffset;

	charOffset = (cfont.width * cfont.nbrows) + 1;  // bytes used by each character
	charOffset = (charOffset * (ch - cfont.offset)) + FONT_HEADER_SIZE;  // char offset (add 4 for font header)
	charWidth  = readFontByte(charOffset);  // get font width from 1st byte
	charOffset++;  // increment pointer to first character data byte

	for (i = 0; i <= charWidth; i++) {  // each font "column" (+1 blank column for spacing)
		h = 0;  // keep track of char height
		for (j = 0; j < cfont.nbrows; j++) 	{  // each column byte
			if (i == charWidth) charData = (uint8_t)0x0; // Insert blank column
			else                charData = readFontByte(charOffset);
			charOffset++;
			
			// Process every row in font character
			for (uint8_t k = 0; k < 8; k++) {
				if (h >= cfont.height ) break;  // No need to process excess bits
				if (bitRead(charData, k)) drawPixel(x + i, y + (j * 8) + k, color);
				else                      drawPixel(x + i, y + (j * 8) + k, _bgColor);
				h++;
			};
		};
	};
	return charWidth;
}

