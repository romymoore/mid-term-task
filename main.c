#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#define bitSet(reg, n) (reg |= (1 << n))
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitCheck(reg, n) (reg >> n & 1)

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

int adc = 0;

ISR(ADC_vect)
{
  adc = ADC;
}

ISR(TIMER1_COMPA_vect)
{
  // this will change between the 3 leds every second
}

int main (){

  usart_init(9600);

  bitSet(ADMUX, REFS0);
  bitSet(ADMUX, MUX1);
  ADCSRA = 7;
  bitSet(ADCSRA, ADIE);
  bitSet(ADCSRA, ADEN);

  TCCR1A = 0;              // normal operation
  TCCR1B = 0;              // reset config
  OCR1A = 15624;          // 1 second match value
  TCCR1B |= (1 << WGM12); // CTC mode
      // prescaler = 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A); // enable interrupt


  unsigned char pwm_value = 0;

  sei();

  _delay_ms(10);

  while (1)
  {
    bitSet(ADCSRA, ADSC);
    usart_send_string(">adc: ");
    usart_send_num(adc, 4, 0);
    usart_send_byte('\n');

    pwm_value = 255 - linear_map(adc, adc_min, adc_max, pwm_min, pwm_max);
   
    OCR0A = pwm_value;

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
