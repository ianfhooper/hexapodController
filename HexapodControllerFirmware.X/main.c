// WaveFlyer Retract Tester
// Written by Ian Hooper (ian@enautic.co)
// (C) Enautic 2024

// Runs on ZEVA EVMS Monitor V3 hardware
// AT90CAN64, 16Mhz external crystal, CKDIV8 off, JTAGEN off

//#define F_CPU 16000000UL // Needed for util/delay.h

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>
#include <stdbool.h>

#include "Touchscreen.h"
#include "compiler.h"

#define RETRACT_RX_CAN_ID       0x18FFEC10  // i.e we send, retract receives
#define LEFT_RETRACT_TX_CAN_ID  0x18FFEC00  // retract sends, we receive
#define RIGHT_RETRACT_TX_CAN_ID 0x18FFEC01

#define MCX_RX_CAN_ID       0x10052001 // for left side, right side is +0x100
#define MCX_TX_CAN_ID       0x10052201

#define BACKLIGHT		(1<<PD4)
#define BACKLIGHT_PORT	PORTD
#define BACKLIGHT_DDR	DDRD

// Name the ADC channels
enum ADCs { V_BATT, LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y };

enum Page {
    MAIN_PAGE // Probably only one page
};

// Different walking types are sent as bitfield, first byte of command message
#define WIGGLE_BIT		0b00000001 // Bit 0 for wiggle vs walk mode
#define HIGH_STEP_BIT	0b00000010 // Bit 1 for normal or high step
#define HIGH_BODY_BIT	0b00000100 // Bit 2 for normal or high body
#define QUICK_STEP_BIT	0b00001000 // Bit 3 for normal or quick (shorter) steps
#define RIPPLE_BIT		0b00010000 // Bit 4 to select ripple gait instead of tripod

// Initialising all the buttons
typedef struct
{
	U16 x, y, width;
	U16 colour;
	const char* text;
	bool highlighted;
    bool selected;
    bool needsRedraw;
    U8 page;
} Button;

enum Buttons {
    NONE = -1,
    WALK_MODE,
    WIGGLE_MODE,
    TRIPOD_GAIT,
    RIPPLE_GAIT,
    LOW_BODY,
    HIGH_BODY,
    LOW_STEP,
    HIGH_STEP,
    LONG_STEP,
    QUICK_STEP,
    RED_EYES,
    GREEN_EYES,
    BLUE_EYES,
    NUM_BUTTONS
};
Button buttons[NUM_BUTTONS];
S8 touchedButton = NONE;

typedef struct
{
    U16 x, y, width;
    U16 colour;
    S8 value, oldValue;
    U8 page;
} Slider;

enum Sliders {
    FRONT_SERVO,
    BACK_SERVO,
    NUM_SLIDERS,
};
Slider sliders[NUM_SLIDERS];

// Function declarations
void Transmit(unsigned char c);
void RenderMainPage();
void HandleTouchDown();
void HandleTouchUp();
void AddDecimalPoint(char* buffer);
void AddDecimalPoint2(char* buffer);
void AddTrailingSpaces(char* buffer, U8 totalLength);
int ReadADC(unsigned char channel);
void SetupPorts();
void InitialiseButton(U8 button, U16 x, U16 y, U16 width, U16 colour, const char* text, U8 selected);
void SelectButton(S8 newButton, S8 oldButton, S8 otherOldButton);
void InitialiseSlider(U8 slider, U16 x, U16 y, U16 width, U16 colour, S8 value, U8 page);
inline void RenderBorderBox(int lx, int ly, int rx, int ry, U16 Fcolor, U16 colour);
void RenderButtons();
void RenderSliders();
void DrawBattery(int x, int y, int percentage);

// Global variables

char buffer[30]; // Used for sprintf functions

short ticks = 0; // For main loop timing

int displayBrightness = 0; // Inverted, 0 means full bright
volatile bool displayNeedsFullRedraw = true;
U8 currentPage = MAIN_PAGE;

short touchTimer;
int touchX, touchY;
S8 buttonPressed = -1;
S8 touchedSlider = -1;

U8 controlBits = 0;

uint8_t left_x, left_y, right_x, right_y; // ADCs of joysticks
int joystick_command_character = 'c'; // For some reason I had to predefine this.. AVR library bug maybe

int controllerSoC = 100;
int hexapodSoC = 100;

inline bool ButtonTouched(Button* button)
{
	return touchX >= button->x-button->width/2 && touchX <= button->x+button->width/2
		&& touchY >= button->y && touchY <= button->y + 32
        && currentPage == button->page;
}

inline bool SliderTouched(Slider* slider)
{
    return touchX >= slider->x-slider->width/2 && touchX <= slider->x+slider->width/2
        && touchY >= slider->y && touchY <= slider->y + 32
        && currentPage == slider->page;
}

ISR(TIMER0_OVF_vect) // Called at 7812Hz, i.e every 2048 cycles of 16Mhz clock
{
	ticks++; // Used for main loop timing

	BACKLIGHT_PORT &= ~BACKLIGHT;
}

ISR(TIMER0_COMP_vect)
{
	if (displayBrightness < 254) // 254 is for 0% night brightness, and 255 is for actually off, but both should have no backlight
		BACKLIGHT_PORT |= BACKLIGHT;	
}

ISR(TIMER1_OVF_vect) // Interrupts at about 30Hz
{
	OCR0A = displayBrightness; // Updates backlight PWM, inverted due to PNP transistor
	
	// Poll touchscreen
	if (Touch_DataAvailable())
	{
		Touch_Read();
		touchTimer++;
		HandleTouchDown();
	}
	else
	{
		if (touchTimer > 0) HandleTouchUp();
		touchTimer = 0;
		touchX = -1;
		touchY = -1;
	}
}

int main(void)
{
	SetupPorts();

    InitialiseButton(WALK_MODE, 140, 30, 100, BLUE, "Walk", true);
    InitialiseButton(WIGGLE_MODE, 260, 30, 100, BLUE, "Wiggle", false);
    
    InitialiseButton(TRIPOD_GAIT, 140, 65, 100, BLUE, "Tripod", true);
    InitialiseButton(RIPPLE_GAIT, 260, 65, 100, BLUE, "Ripple", false);
    
    InitialiseButton(LOW_BODY, 140, 100, 100, BLUE, "Low", true);
    InitialiseButton(HIGH_BODY, 260, 100, 100, BLUE, "High", false);
    
    InitialiseButton(LOW_STEP, 140, 135, 100, BLUE, "Low", true);
    InitialiseButton(HIGH_STEP, 260, 135, 100, BLUE, "High", false);
    
    InitialiseButton(LONG_STEP, 140, 170, 100, BLUE, "Long", true);
    InitialiseButton(QUICK_STEP, 260, 170, 100, BLUE, "Quick", false);
        
    InitialiseButton(RED_EYES, 122, 205, 64, RED, "Red", false);
    InitialiseButton(GREEN_EYES, 200, 205, 70, GREEN, "Green", true);
    InitialiseButton(BLUE_EYES, 278, 205, 64, BLUE, "Blue", false);
   
    
	_delay_ms(100*16); // Wait for LCD to power up - for some reason delay function not recognising F_CPU
	TFT_Init();

	Touch_Init();

	sei(); // Enable interrupts

	while (1)
	{
		switch (buttonPressed)
        {
            case NONE:
                break;
                
            case WALK_MODE:
                controlBits &= ~WIGGLE_BIT;
                SelectButton(WALK_MODE, WIGGLE_MODE, NONE);
                break;
                
            case WIGGLE_MODE:
                controlBits |= WIGGLE_BIT;
                SelectButton(WIGGLE_MODE, WALK_MODE, NONE);
                break;
                
            case TRIPOD_GAIT:
                controlBits &= ~RIPPLE_BIT;
                SelectButton(TRIPOD_GAIT, RIPPLE_GAIT, NONE);
                break;
              
            case RIPPLE_GAIT:
                controlBits |= RIPPLE_BIT;
                SelectButton(RIPPLE_GAIT, TRIPOD_GAIT, NONE);
                break;
                
            case LOW_BODY:
                controlBits &= ~HIGH_BODY_BIT;
                SelectButton(LOW_BODY, HIGH_BODY, NONE);
                break;
                
            case HIGH_BODY:
                controlBits |= HIGH_BODY_BIT;
                SelectButton(HIGH_BODY, LOW_BODY, NONE);
                break;
                
            case LOW_STEP:
                controlBits &= ~HIGH_STEP_BIT;
                SelectButton(LOW_STEP, HIGH_STEP, NONE);
                break;
                
            case HIGH_STEP:
                controlBits |= HIGH_STEP_BIT;
                SelectButton(HIGH_STEP, LOW_STEP, NONE);
                break;
                
            case LONG_STEP:
                controlBits &= ~QUICK_STEP_BIT;
                SelectButton(LONG_STEP, QUICK_STEP, NONE);
                break;
                
            case QUICK_STEP:
                controlBits |= QUICK_STEP_BIT;
                SelectButton(QUICK_STEP, LONG_STEP, NONE);
                break;
                
            case RED_EYES:
                Transmit('r');
                SelectButton(RED_EYES, GREEN_EYES, BLUE_EYES);
                break;
                
            case GREEN_EYES:
                Transmit('g');
                SelectButton(GREEN_EYES, RED_EYES, BLUE_EYES);
                break;
                
            case BLUE_EYES:
                Transmit('b');
                SelectButton(BLUE_EYES, RED_EYES, GREEN_EYES);
                break;
        }
        buttonPressed = NONE;
        
        // Check for received bytes
        if (UCSR1A & (1<<RXC1))
        {
            int newHexapodSoC = UDR1;
            if (newHexapodSoC < hexapodSoC) hexapodSoC = newHexapodSoC; // Expect to always go down, avoids jiggling
        }
        
        while (ticks > 781) // 10Hz
		{
			ticks -= 781;
           
            // Batt voltage via 10K:10K divider, 0-1023 ADC for 0-6.6V, works out 650 ADC for 4.2V, 500 ADC for 3.2V
            int newControllerSoC = (ReadADC(V_BATT)-500)*2/3;
            if (newControllerSoC < controllerSoC)
                controllerSoC = newControllerSoC; // Always decreases, avoids jiggling due to sampling noise
                    
            left_x = ReadADC(LEFT_X)>>2; // Downsample to 8 bit
            left_y = ReadADC(LEFT_Y)>>2;
            right_x = ReadADC(RIGHT_X)>>2;
            right_y = ReadADC(RIGHT_Y)>>2;
            Transmit(joystick_command_character);
            Transmit(controlBits);
            Transmit(left_x);
            Transmit(left_y);
            Transmit(right_x);
            Transmit(right_y);
            Transmit(controlBits + left_x + left_y + right_x + right_y); // Basic checksum
		}
        
        switch (currentPage)
        {
            case MAIN_PAGE:
                RenderMainPage(); // Only one page for Hexapod?
                break;
        }
        RenderButtons();
        RenderSliders();
        displayNeedsFullRedraw = false;
	}
    return 0; // Never gets here but compiler wants to see it
}

void Transmit(unsigned char c)
{
    while (( UCSR1A & (1<<UDRE1)) == 0); // Wait until TX buffer is empty (if needed)
    UDR1 = c;// Putting data into buffer sends the data
    while (( UCSR1A & (1<<UDRE1)) == 0);
    _delay_ms(2); // Dirty hack, otherwise seems to be some bug with sending successive characters
}

void RenderMainPage()
{
    if (displayNeedsFullRedraw)
    {
        TFT_Fill(BLACK);
        TFT_Text("Ian's Hexapod", 2, 3, 1, BLUE, BLACK);
        TFT_Text("C", 258, 3, 1, L_GRAY, BLACK);
        TFT_Text("H", 182, 3, 1, L_GRAY, BLACK);
        
        TFT_Text("Mode:", 2, 36, 1, WHITE, BLACK);
        TFT_Text("Gait:", 2, 71, 1, WHITE, BLACK);
        TFT_Text("Body:", 2, 106, 1, WHITE, BLACK);
        TFT_Text("Step:", 2, 141, 1, WHITE, BLACK);
        TFT_Text("Eyes:", 2, 211, 1, WHITE, BLACK);        
        TFT_Box(0, 24, 320, 25, L_GRAY);
    }
    
    DrawBattery(200, 5, hexapodSoC);
    DrawBattery(276, 5, controllerSoC);
}

void HandleTouchDown()
{
	touchX = Touch_GetX();
	touchY = Touch_GetY();

	if (touchTimer == 3)
	{
        for (U8 n=0; n<NUM_BUTTONS; n++)
            if (ButtonTouched(&buttons[n])) touchedButton = n;
       
        for (U8 n=0; n<NUM_SLIDERS; n++)
            if (SliderTouched(&sliders[n])) touchedSlider = n;
	}
    else if (touchTimer > 3) // could be dragging a slider
    {
        for (U8 n=0; n<NUM_SLIDERS; n++)
        {
            Slider* slider = &sliders[n];
            if (SliderTouched(&sliders[n]) && touchedSlider == n)
            {
                int usableWidth = slider->width-16;
                slider->value = 100*(touchX - (slider->x-usableWidth/2))/usableWidth; // Get a percentage
                if (slider->value < 0) slider->value = 0;
                if (slider->value > 100) slider->value = 100;
            }
        }
    }
}

void HandleTouchUp()
{
	if (touchTimer < 3) return; // Ignore too-fast touches

    if (ButtonTouched(&buttons[touchedButton])) // Only accept if finger still within button area
        buttonPressed = touchedButton; // TODO: Ambiguous variable names?
	touchedButton = NONE;
    touchedSlider = NONE;
}

void SetupPorts()
{
	// TODO: Move this into Touchscreen.c?
	DP_Lo_DDR = 0b11111111; // DP_Lo
	DP_Hi_DDR = 0b11111111; // DP_Hi
	BACKLIGHT_DDR |= BACKLIGHT;

	DDRG |= RST;
	DDRD |= RS + CS + WR; // RD not used

	// Timer0 used for ticks and display backlight PWM
	TCCR0A = (1<<CS01) /* + (1<<WGM01) + (1<<WGM00) + (1<<COM0A1) */ ; // clk/8 counting rate = 1Mhz, overflows at 7812Hz, PWM OFF - was fast PWM, non inverting
	TIMSK0 = (1<<TOIE0) + (1<<OCIE0A); // Interrupt on overflow and output compare

	// Timer 1 used for touchscreen polling interrupt
	TCCR1B = (1<<CS11); // Timer running with clk/8 prescaler -> about 30hz overflows at 16Mhz clock
	TIMSK1 = (1<<TOIE1); // Enable interrupt on overflow
    
    // Initialise UART
    UBRR1H = 0;
    UBRR1L = 103; // From AT90CAN32/64/128 manual, page 202
    
    // Set frame format: 8data, no parity, 1 stop bits
    // UMSEL1 = 0 for asynchronous, UPM11 and 10 = 0 for no parity, UCSZ10 and UCSZ11 = 1 for 8-bit
    // USBS1 = 0 for 1 stop bit
    UCSR1C =  (1<<UCSZ10) | (1<<UCSZ11);
    
    UCSR1B = /*(1<<TXCIE1) | (1<<RXCIE1) |*/ (1<<RXEN1) | (1<<TXEN1); // Enable TX and RX, and interrupt bits
    
    ADMUX = (1<<REFS0); // AVCC reference, capacitor on AREF pin
    ADCSRA = 0b10000111; // ADEN plus prescaler bits 111 = /128 (gives 125kHz ADC clock, needs to be 50-200kHz)
    
}

int ReadADC(unsigned char channel)
{
	ADMUX = channel | (1<<REFS0);
	_delay_us(10); // Delay needed for the stabilization of the ADC input voltage
	ADCSRA |= (1<<ADSC); // Start the AD conversion
	while ((ADCSRA & 0x10) == 0); // Wait for the AD conversion to complete
	ADCSRA |= 0x10;
	return ADCW;
}

void InitialiseButton(U8 button, U16 x, U16 y, U16 width, U16 colour, const char* text, U8 selected)
{
    Button* buttonPointer = &buttons[button];
    buttonPointer->x = x;
    buttonPointer->y = y;
    buttonPointer->width = width;
    buttonPointer->colour = colour;
    buttonPointer->text = text;
    buttonPointer->highlighted = false;
    buttonPointer->selected = selected;
    buttonPointer->needsRedraw = true;
    buttonPointer->page = MAIN_PAGE; // Hexapod controller is single page display
}

void SelectButton(S8 newButton, S8 oldButton, S8 otherOldButton)
{
    buttons[newButton].selected = true;
    buttons[newButton].needsRedraw = true;
    buttons[oldButton].selected = false;
    buttons[oldButton].needsRedraw = true;
    if (otherOldButton != NONE)
    {
        buttons[otherOldButton].selected = false;
        buttons[otherOldButton].needsRedraw = true;
    }
}

void InitialiseSlider(U8 slider, U16 x, U16 y, U16 width, U16 colour, S8 value, U8 page)
{
    Slider* sliderPointer = &sliders[slider];
    sliderPointer->x = x;
    sliderPointer->y = y;
    sliderPointer->width = width;
    sliderPointer->colour = colour;
    sliderPointer->value = value;
    sliderPointer->oldValue = -1; // Force initial redraw
    sliderPointer->page = page;    
}

inline void RenderBorderBox(int lx, int ly, int rx, int ry, U16 Fcolor, U16 colour)
{
	TFT_Box(lx, ly, rx, ry, Fcolor);
	TFT_Box(lx+2, ly+2, rx-2, ry-2, colour);
}

void RenderButtons()
{
    for (U8 n=0; n<NUM_BUTTONS; n++)
    {
        Button* button = &buttons[n];
        
        bool wasHighlighted = button->highlighted;
        button->highlighted = (ButtonTouched(button) && touchedButton == n);
        
        if (button->page == currentPage && (wasHighlighted != button->highlighted || button->needsRedraw))
        {
            U16 colour = (button->highlighted || button->selected) ? button->colour : BLACK;
            RenderBorderBox(button->x - button->width/2, button->y, button->x + button->width/2, button->y+28, button->colour, colour);
            int textColour = WHITE;
            if (colour == D_GRAY) textColour = D_GRAY;
            TFT_CentredText(button->text, button->x, button->y+6, 1, textColour, colour);
            
            button->needsRedraw = false;
        }
	}
}

void RenderSliders()
{
    // Test slider first
    
    for (U8 n=0; n<NUM_SLIDERS; n++)
    {
        Slider* slider = &sliders[n];
        
        if (slider->page == currentPage && slider->oldValue != slider->value)
        {
            int usableWidth = slider->width-16;
            int middle = slider->x-usableWidth/2+usableWidth*slider->value/100;
            
            TFT_Box(slider->x-slider->width/2, slider->y, slider->x+slider->width/2, slider->y+8, BLACK);
            TFT_Box(slider->x-slider->width/2, slider->y+8, slider->x+slider->width/2, slider->y+24, D_GRAY);
            TFT_Box(slider->x-slider->width/2, slider->y+24, slider->x+slider->width/2, slider->y+32, BLACK);
            TFT_Box(middle-8, slider->y, middle+8, slider->y+32, slider->colour);
            slider->oldValue = slider->value;
        }
    }
}

void DrawBattery(int x, int y, int percentage)
{
    if (displayNeedsFullRedraw)
    {
        TFT_Box(x, y, x+34, y+12, L_GRAY);
        TFT_Box(x+34, y+4, x+36, y+8, L_GRAY);
    }
    
    U16 colour = GREEN;
    if (percentage < 20)
        colour = RED;
    else if (percentage < 50)
        colour = YELLOW;
    
    int width = percentage*30/100;
    if (width<3) width = 3; // Show bit of red even when flat
    
    TFT_Box(x+2, y+2, x+2+width, y+10, colour);
    TFT_Box(x+2+width+1, y+2, x+32, y+10, BLACK);
}