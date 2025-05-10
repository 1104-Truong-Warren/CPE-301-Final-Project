#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include <DHT.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>

// ----- Timer Register Definitions -----
// Direct define helps with memory allocation / Doesn't change at runtime
// Pin 11/12/13
#define TCCR1A _SFR_MEM8(0x80) 
#define TCCR1B _SFR_MEM8(0x81)
#define TCNT1  _SFR_MEM16(0x84)
#define TIMSK1 _SFR_MEM8(0x6F)

// Pin 9/10
#define TCCR2A _SFR_MEM8(0xB0)
#define TCCR2B _SFR_MEM8(0xB1)
#define OCR2A  _SFR_MEM8(0xB3)
#define TIMSK2 _SFR_MEM8(0x70)

// ----- System Constants -----
#define LCD_LEN     16    
#define LCD_RS      12
#define LCD_EN      11
#define LCD_D4      6
#define LCD_D5      5
#define LCD_D6      4
#define LCD_D7      3

#define DHT_PIN     7           // Heat sensor to pin 7
#define DHTTYPE     DHT11       
#define REV_STEPS   2038
#define FAN_MASK    (1 << PB4)  // Fan control  pin 10
#define SPEAKER_PIN (1 << PB6)  // Speaker/Buzzer control pin 12
#define TEMP_LOW    50          // Temp control low/med/high
#define TEMP_MEDIUM 65
#define TEMP_HIGH   75
#define START_BTN   (1 << PB3)  // Buttons controls pin 50
#define RESET_BTN   (1 << PB2)  // Pin 51
#define CTRL_BTN    (1 << PB1)  // Pin 52

// ----- Function Prototypes -----
void UART0_Init(unsigned long baud);
void UART0_PutChar(char c);
void UART0_PutStr(const char *s);
void setup_sound();
void update_sound(float tempF);
void handleRunning(float tempF);
void handleIdle(float tempF);
void handleError();
void handleDisabled();
void checkStateTransition(DateTime now);
unsigned int ADC_Read(unsigned char ch);

// ----- Global Variables -----
// Controls which state the Arudino is in
typedef enum STATE {DISABLED, IDLE, ERROR, RUNNING} STATE;
volatile STATE dev_state = DISABLED;
volatile STATE prev_state = DISABLED;


LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Stepper step(REV_STEPS, 28, 26, 24, 22);
DHT dht(DHT_PIN, DHTTYPE);
RTC_DS3231 rtc;

char lcd_buf[LCD_LEN], err_msg[LCD_LEN];
const char state_map[4][16] = {"(DISABLED)", "IDLE", "ERROR", "RUNNING"};
const uint8_t led_mask_map[4] = {0x20, 0x80, 0x08, 0x02};

const uint16_t temp_threshold = 42, wtr_threshold = 400;
volatile uint16_t wtr_level = 0;
volatile uint32_t update_timer = 0; // Acts like millis()
volatile uint8_t btn_flag = 0;

// Musical notes (C4, D4, E4, F4)
const uint16_t temp_notes[] = {261, 294, 330, 349}; // Frequncy of the sound
volatile uint16_t current_note = 0;
volatile uint8_t sound_active = 0;

// ----- Timer ISRs -----
ISR(TIMER1_OVF_vect) {
    if(sound_active && (dev_state == RUNNING)) {
        PORTB ^= SPEAKER_PIN;
        TCNT1 = 65535 - current_note;
    } else {
        PORTB &= ~SPEAKER_PIN;
    }
}

ISR(TIMER2_COMPA_vect) {
    update_timer++; // increment every 1ms
}

ISR(PCINT0_vect) {
    btn_flag = 1;
}

// ----- UART Functions -----
void UART0_Init(unsigned long baud) {
    uint16_t ubbr = (F_CPU / 16 / baud - 1);
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    UBRR0 = ubbr;
}

void UART0_PutChar(char c) {
    while(!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void UART0_PutStr(const char *s) {
    // Keeps looping until the last char
    while(*s != '\0') UART0_PutChar(*s++);
}

// ----- Sound Functions -----
void setup_sound() {
    DDRB |= SPEAKER_PIN;
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = (1 << TOIE1);
}

void update_sound(float tempF) {
    if(dev_state != RUNNING) {
        sound_active = 0;
        return;
    }

    uint16_t frequency = temp_notes[3]; // Default to warning tone
    // Makes a different sound depending on temp
    if(tempF < TEMP_LOW) frequency = temp_notes[0];
    else if(tempF < TEMP_MEDIUM) frequency = temp_notes[1];
    else if(tempF < TEMP_HIGH) frequency = temp_notes[2];

    current_note = (F_CPU / (2UL * 64 * frequency)) - 1;
    
    if(!sound_active) {
        TCNT1 = 65535 - current_note;
        TCCR1B = (1 << CS11) | (1 << CS10);
        sound_active = 1;
    }
}

// ----- State Handlers -----
void handleDisabled() {
    PORTB &= ~FAN_MASK;
}

// Idle
void handleIdle(float tempF) {
    PORTB &= ~FAN_MASK;
    if(tempF >= temp_threshold) dev_state = RUNNING;
    if(wtr_level < wtr_threshold) {
        strcpy(err_msg, "Low water!");
        dev_state = ERROR;
    }
}

// Error msg
void handleError() {
    PORTB &= ~FAN_MASK;
    lcd.setCursor(0, 0);
    lcd.print(err_msg);
}

// Running
void handleRunning(float tempF) {
    PORTB |= FAN_MASK;
    update_sound(tempF);
    
    if(tempF < temp_threshold) dev_state = IDLE;
    if(wtr_level < wtr_threshold) {
        strcpy(err_msg, "Low water!");
        dev_state = ERROR;
    }
}

// ----- ADC Function -----
unsigned int ADC_Read(unsigned char ch) {
    ADMUX = (1 << REFS0) | (ch & 0x07);
    ADCSRA |= (1 << ADSC);
    while(ADCSRA & (1 << ADSC));
    return ADC;
}

// ----- Main Program -----
void setup() {
    lcd.begin(16, 2);
    dht.begin();
    rtc.begin();
    step.setSpeed(2);
    
    // Initialize I/O
    DDRC |= 0xAA;  // LED outputs
    DDRB |= FAN_MASK;
    PORTB |= (START_BTN | RESET_BTN | CTRL_BTN); // Enable pull-ups
    
    // Peripheral initialization
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    setup_sound();
    UART0_Init(9600);

    // Timer2 setup (1ms overflow)
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22); // prescaler = 64
    OCR2A = 249;          // for 1ms with 16MHz clock
    TIMSK2 |= (1 << OCIE2A);

    // Interrupts
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (START_BTN | RESET_BTN);
    sei();
    
    lcd.print("Ready");
}

void loop() {
    static uint32_t last_update = 0;
    static uint32_t last_btn_time = 0;

    DateTime now = rtc.now();
    float tempF = dht.readTemperature(true);
    
    if(isnan(tempF)) {
        // Using string copy to show msg (strcpy)
        strcpy(err_msg, "Sensor Error!");
        dev_state = ERROR;
    }

    wtr_level = ADC_Read(0);

    if(update_timer - last_update >= 60000) {
        snprintf(lcd_buf, LCD_LEN, "T:%dF H:%d%%", 
                 (int)tempF, (int)dht.readHumidity());
        last_update = update_timer;
    }

    if(btn_flag) {
        btn_flag = 0;
        if((update_timer - last_btn_time) > 200) {
            last_btn_time = update_timer;
            if(!(PINB & RESET_BTN) && (dev_state == ERROR)) {
                if(ADC_Read(0) > wtr_threshold) dev_state = IDLE; // If water level too high set to Idle
            }
            if(!(PINB & START_BTN)) {
                dev_state = (dev_state == DISABLED) ? IDLE : DISABLED;
            }
        }
    }

    lcd.setCursor(0, 0);
    lcd.print(lcd_buf);
    lcd.setCursor(0, 1);
    lcd.print(state_map[dev_state]);
    
    PORTC = led_mask_map[dev_state];

    switch(dev_state) {
        case DISABLED: handleDisabled(); 
        break;

        case IDLE: handleIdle(tempF); 
        break;

        case ERROR: handleError(); 
        break;

        case RUNNING: handleRunning(tempF); 
        break;
    }

    checkStateTransition(now);
}

void checkStateTransition(DateTime now) {
    char log_msg[50];

    if(prev_state != dev_state) {
        snprintf(log_msg, sizeof(log_msg), 
               "State %s -> %s @ %02d:%02d:%02d\n",
               state_map[prev_state], state_map[dev_state],
               now.hour(), now.minute(), now.second());

        UART0_PutStr(log_msg);
        prev_state = dev_state;

        lcd.clear();
    }
}
