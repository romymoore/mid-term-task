// last digit = odd: larger distance = lower brightness
// Last four digits: 9243

// Calculate OCR0A's value ranging in 100 and 255. Use x [0, 9999] in a positive linear mapping.
// OCR0A = ((x - x1)*(y2-y1))/(x2-1) + y1
// OCR0A = ((9243 - 0)*(255-100))/(9999-0) + 100
// OCR0A = 243

// Therefore, Tov is calculated using the formula
// Tov = (TOP + 1)*(P/CLK_FREQ), where TOP = 243 (OCR0A), P = 1 and CLK_FREQ = 16e6
// Thus, since P = 1 the formula comes down to: Tov = 244/16e6 = 15.25e-6 us

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define bitSet(reg, n) (reg |= 1 << n)
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitCheck(reg, n) (reg >> n & 1)

#define start_tc0 (TCCR0B |= 0b001)
#define stop_tc0 (TCCR0B &= 0b000)

#define enable_tc0_int (bitSet(TIMSK0, TOIE0))
#define disable_tc0_int (bitClear(TIMSK0, TOIE0))

#define pin_trigger PD4
#define pin_echo PD7
#define pin_oc0b PD5

#define OCR0A_TOP 196.03
#define prescaler 1
#define MAX_CLK 255

float Tov;
float clock_tc0;

volatile unsigned int numOV = 0;
volatile unsigned int numOV_max_sonar = 0;

volatile uint8_t *ddr_sonar = &DDRD;
volatile uint8_t *port_sonar = &PORTD;
volatile uint8_t *pin_sonar = &PIND;

float vel_sound = 343;

void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(char *pstr);
void usart_send_num(float num, char num_int, char num_decimal);

float fun_map(float x, float x1, float x2, float y1, float y2);
float sonar(void);
void my_delay_us(unsigned long x);

void usart_init(float baud)
{
    UBRR0H = 0;
    UBRR0L = 103;   // 9600 baud at 16MHz
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    sei();
}

void usart_send_byte(unsigned char data)
{
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void usart_send_string(char *pstr)
{
    while (*pstr)
    {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *pstr++;
    }
}

void usart_send_num(float num, char num_int, char num_decimal)
{
  char buf[20];
  dtostrf(num, num_int, num_decimal, buf);
  usart_send_string(buf);
}

ISR(TIMER0_OVF_vect)
{
  numOV++;
}

float fun_map(float x, float x1, float x2, float y1, float y2)
{
  if (x < x1) x = x1;
  if (x > x2) x = x2;
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

void config_tc0()
{
  TCCR0A = 0;
  TCCR0B = 0;
  clock_tc0 = 16.0e6 / prescaler; // clock frequency
  TCCR0A |= 0b11; //fast pwm-top mode: WGM01 & WGM00 = 1
  TCCR0B |= 0b1000; // WGM02 = 1
  bitSet(TCCR0A, COM0B1);
  OCR0A = OCR0A_TOP;
  OCR0B = 0;

  Tov = ((OCR0A + 1) / clock_tc0) * 1e6; //in us

  numOV_max_sonar = 12.0 / vel_sound / Tov * 1.0e6;
}

int main(void)
{
  usart_init(9600);

  bitSet(DDRD, pin_oc0b);

  bitSet(*ddr_sonar, pin_trigger);
  bitClear(*ddr_sonar, pin_echo);

  config_tc0();
  start_tc0;
  sei();

  while (1)
  {
    float D = sonar(); // measure distance using sonar
    usart_send_num(D, 3, 3); // Print out sonar results for debugging purposes
    usart_send_byte(';');

    // Map sonar results to OCR0B, meaning PWM fast with OCR0B used for duty cycle
    OCR0B = fun_map(D, 10, 300, OCR0A_TOP - 1, 10);

    // Print out OCR0B for checking duty cycle
    usart_send_num(OCR0B, 3, 0);
    usart_send_byte('\n');
    usart_send_string(">Dmm:");
    my_delay_us(100e3);

  }
}

float sonar(void)
{
  bitClear(*port_sonar, pin_trigger);
  my_delay_us(2);
  bitSet(*port_sonar, pin_trigger);
  my_delay_us(11);
  bitClear(*port_sonar, pin_trigger);

  while (!bitCheck(*pin_sonar, pin_echo))
    ;
 
  enable_tc0_int;
  numOV = 0;
  TCNT0 = 0;
  while (bitCheck(*pin_sonar, pin_echo) && numOV < numOV_max_sonar)
    ;
  disable_tc0_int;

  uint8_t tmp = TCNT0;

  float tElapse = (numOV)*Tov + (float)(tmp+1) / clock_tc0 * 1.0e6;
  float Dmm = tElapse / 1.0e6 * vel_sound / 2. * 1000.;

  return Dmm;
}

void my_delay_us(unsigned long x)
{
  // Check how many overflows will be needed for this delay
  unsigned long numOV_max = (float)x / Tov;

  // Need to know precisely how many clock cycles we need still to delay x us
  unsigned char tcnt0_max = ((float)x - numOV_max * Tov) / 1.0e6 * clock_tc0;

  // Monitor the TC0 to perform numOV_max times of overflows
  if (numOV_max > 0)
  {
    numOV = 0;
    TCNT0 = 0;
    enable_tc0_int;
    while (numOV < numOV_max)
      ;
    disable_tc0_int;
  }

  // Let the TC0 also delay tcnt0_max numbers of clock cycles
  TCNT0 = 0;
  while (TCNT0 < tcnt0_max)
    ;
}
