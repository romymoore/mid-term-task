#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#define bitSet(reg, n) (reg |= (1 << n))
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitCheck(reg, n) (reg >> n & 1)

#define red_light PB0
#define yellow_light PB1
#define green_light PB2

#define sonar_trigger PB3
#define sonar_echo PB4

#define adc_min 0
#define adc_max 1023
#define pwm_min 0
#define pwm_max 255
#define linear_map(x, in_min, in_max, out_min, out_max) \
(((long)(x) - (long)(in_min)) * ((long)(out_max) - (long)(out_min)) / ((long)(in_max) - (long)(in_min)) + (long)(out_min))

void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(const char* pstr);
void usart_send_num(float num, char num_int, char num_decimals);

volatile int adc = 0;
volatile unsigned char led_state = 0;
// led_state: 0 = red, 
// led_state: 1 = yellow
// led_state: 2 = green
// led_state: 3 = object detected, stay red until object is out of range
// led_state: 5 = manual red
// led_state: 6 = manual yellow
// led_state: 7 = manual green
// led_state: 8 = emergency green

ISR(ADC_vect)
{
  adc = ADC;  // Read ADC value and store in global variable
}

ISR(TIMER1_COMPA_vect)
{
  // this ISR changes between the 3 leds each second when in auto mode
  if(led_state == 0)
  {
    DDRB = 0b00001000;
    bitSet(DDRB, red_light);
    led_state = 1;
  }
  else if(led_state == 1)
  {
    DDRB = 0b00001000;
    bitSet(DDRB, yellow_light);
    led_state = 2;
  }
  else if(led_state == 2)
  {
    DDRB = 0b00001000;
    bitSet(DDRB, green_light);
    led_state = 0;
  }
}

int main (){

  usart_init(9600);

  _delay_ms(10);

  // PWM for LEDs | timer0
  DDRD |= 1 << PD6;                                         // Sets PD6 as output pin
  TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);     // Fast PWM mode, non-inverting
  TCCR0B = (1 << CS01) | (1 << CS00);                       // Prescaler = 64                   

  // ADC setup
  bitSet(ADMUX, REFS0);                                     // Sets AVcc reference as 5V
  bitSet(ADMUX, MUX1);                                      // Select ADC2 (PC2) as input                    
  ADCSRA = 7;                                               // Set ADC clock prescaler to 128 (16MHz/128 = 125KHz)       
  bitSet(ADCSRA, ADIE);                                     // Enable ADC interrupt
  bitSet(ADCSRA, ADEN);                                     // Enable ADC module

  // 1 second timer | timer1
  TCCR1A = 0;                                               // Clear control register A, normal mode
  TCCR1B = 0;                                               // Clear control register B    
  OCR1A = 15624;                                            // sets compare value (16MHz/1024 prescaler = 15625 counts per second)
  TCCR1B |= (1 << WGM12);                                   // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);                      // Prescaler = 1024
  TIMSK1 |= (1 << OCIE1A);                                  // Enable timer compare interrupt 

  sei();

  unsigned char pwm_value = 0;

  int count = 0;
  int vel_sound = 343;
  int timeout = 30000;

  while (1)
  {
    bitSet(ADCSRA, ADSC);
    usart_send_string(">adc: ");
    usart_send_num(adc, 4, 0);
    usart_send_byte('\n');

    pwm_value = 255 - linear_map(adc, adc_min, adc_max, pwm_min, pwm_max);
   
    OCR0A = pwm_value;

    count = 0;
    timeout = 30000;

    bitClear(PORTB, sonar_trigger);
    _delay_us(2);
    bitSet(PORTB, sonar_trigger);
    _delay_us(11);
    bitClear(PORTB, sonar_trigger);

    while(!bitCheck(PINB, sonar_echo));

    while(bitCheck(PINB, sonar_echo) && timeout--)
    {
      count++;
      _delay_us(1);
    }

    float Distance = (float)count / 1.0e6 * vel_sound / 2. *1000.;
 
    usart_send_string(">Distance:");
    usart_send_num(Distance, 6, 6);
    usart_send_byte('\n');

    if(Distance < 200)
    {
      led_state = 3;
      DDRB = 0b00001000;
      bitSet(DDRB, red_light);
    }
    else if(led_state == 3 && Distance >= 200)
    {
      led_state = 0;
    }

    _delay_ms(10); 

  }

}

void usart_init(float baud)
{
  float ubrr0 = 1.0e6 / baud - 1;

  int ubrr0a = (int)ubrr0;

  if(ubrr0 - ubrr0a >= 0.5)
  {
    ubrr0a++;
  }

  UBRR0 = ubrr0a;
  bitSet(UCSR0B, TXEN0);
  UCSR0C |= 3 << UCSZ00;

}

void usart_send_byte(unsigned char data)
{

  while (!bitCheck(UCSR0A, UDRE0));
  UDR0 = data;

}

void usart_send_string(const char *pstr)
{
  while (*pstr != '\0')
  {
    usart_send_byte(*pstr);
    pstr++;
  }
}

void usart_send_num(float num, char num_int, char num_decimals)
{
  char wholebuf[num_int + 1];
  char fracbuf[num_decimals + 1];

  int whole = (int)num;
  float frac = (num - whole) * pow(10, num_decimals);

  sprintf(wholebuf, "%d", whole);
  usart_send_string(wholebuf);
  usart_send_byte('.');

  sprintf(fracbuf, "%d", (int)frac);
  usart_send_string(fracbuf);

}
