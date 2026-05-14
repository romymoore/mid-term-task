#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#define bitSet(reg, n) (reg |= (1 << n))
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitToggle(reg, n) (reg ^= (1 << n))
#define bitCheck(reg, n) (reg >> n & 1)

#define pin_pwm PD3

volatile float tLow;
volatile float tHigh;

float tFall = 0;
float tRise = 0;

volatile float Tov;
volatile float Tclk_tc1;

bool flag_captured = 0;
volatile uint16_t numOV = 0;
uint16_t icr1;

void set_tc1();
void set_tc2();

// Function prototypes for USART
void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(const char* pstr);
void usart_send_num(float num, char num_int, char num_decimal);
void usart_init_v2(float baud);
void usart_flush(void);

ISR(TIMER1_OVF_vect)
{
  numOV++;
}

ISR(TIMER1_CAPT_vect)
{
  icr1 = ICR1 + 1;
  float tmp = numOV * Tov + icr1 * Tclk_tc1;

  if(!bitCheck(TCCR1B, ICES1))
  {
    tFall = tmp;
    tHigh = tFall - tRise;
  }
  else
  {
    tRise = tmp;
    tLow = tRise - tFall;
  }

  bitToggle(TCCR1B, ICES1);
}

int main()
{
  bitClear(DDRD, PD6); // Set PD6 as input
  bitClear(DDRD, PD7); // Set PD7 as input
  
  bitSet(DDRD, pin_pwm); // Set PD3 as output
  bitSet(ACSR, ACIC);

  usart_init(115200);

  set_tc2();
  set_tc1();

  sei();

  float dc_est = 0;
  float dc_real = (OCR2B = 1.)/(OCR2A +1.) * 100.;

  unsigned int cnt = 0;

  while (1)
  {
    if (tHigh !=0 && tLow != 0)
    {
     // dc_est = tLow / (tLow - tLow-old) * 100;
     dc_est = tHigh / (tHigh + tLow) * 100;
    }

    usart_send_string(">dc_est(%):");
    usart_send_num(dc_est, 3, 3);
    usart_send_byte('\n');
    usart_send_string(">dc_real(%):");
    usart_send_num(dc_real, 3, 3);
    usart_send_byte('\n');
    usart_send_string(">tHigh:");
    usart_send_num(tHigh, 6, 3);
    usart_send_byte('\n');
    usart_send_string(">tLow:");
    usart_send_num(tLow, 6, 3);
    usart_send_byte('\n');

    _delay_ms(10);

    if (cnt++ % 100 == 0)
    {
      OCR2B++;
      dc_real = (OCR2B + 1.)/(OCR2A + 1.) * 100.;
      if (OCR2B == OCR2A)
      {
        OCR2B = 0;
      }
    }
  }
  return 0;
}

void set_tc1()
{
  bitSet(TCCR1B, ICES1); // Capture on rising edge
  bitSet(TCCR1B, ICNC1); // Enable noise canceler
  bitSet(TIMSK1, TOIE1); // Enable overflow interrupt
  bitSet(TIMSK1, ICIE1); // Enable input capture interrupt
  bitSet(TCCR1B, CS12);
  bitClear(TCCR1B, CS11);
  bitSet(TCCR1B, CS10);

  float f = 16.e6 / 1024; // Timer frequency
  Tov = 65536. / f; // Time for one overflow
  Tclk_tc1 = 1./f; // Time for one timer tick
}

void set_tc2()
{
  TCCR2A |= 1 << WGM21 | 1 << WGM20; // Fast PWM mode
  TCCR2B |= 1 << WGM22; // Fast PWM mode with OCR2A as top
  TCCR2A |= (1 << COM2B1); // Clear OC2B on compare match, set OC2B at BOTTOM (non-inverting mode)

  OCR2A = 199;
  OCR2B = 9;

  TCCR2B |= 1 << CS22 | 1 << CS21 | 1 << CS20; // Prescaler 1024
}


void usart_init_v2(float baud)
{
  usart_init(baud);
  bitSet(UCSR0B, RXEN0);
}

void usart_flush(void)
{
  while(bitCheck(UCSR0A, RXC0))
  {
    (void)UDR0;
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
