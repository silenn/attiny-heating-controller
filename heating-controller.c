/**********************************************************/
/*
*    Heating Controller
*
* @author       Silenn
* @date         03-Mar-2023
* @version      v.1.0
*/
/**********************************************************/
/* Default connections for attiny13a:

                           ATTINY13A
                          ___________
            (RESET) PB5 -|1   \_/   8|- VCC
    LOAD1 - (CLKI)  PB3 -|           |- PB2 (SCK/ADC1/T0)  - SENSOR
    LOAD2 - (INT4)  PB4 -|           |- PB1 (MISO/AIN1)    - LED
                    GND -|___________|- PB0 (MOSI)            
*/
/**********************************************************/
/* Default connections for atmega8a:

                           ATMEGA8A
                          ___________
          (RESET)   PC6 -|1   \_/  28|- PC5 (ADC5/SCL)
          (RXD)     PD0 -|           |- PC4 (ADC4/SDA)
 DBG UART (TXD)     PD1 -|           |- PC3 (ADC3)
          (INT0)    PD2 -|           |- PC2 (ADC2)
          (INT1)    PD3 -|           |- PC1 (ADC1)
          (XCK/T0)  PD4 -|           |- PC0 (ADC0)
                    VCC -|           |- GND
                    GND -|           |- AREF
          (OSC1)    PB6 -|           |- AVCC
          (OSC2)    PB7 -|           |- PB5 (SCK)      - LED
          (T1)      PD5 -|           |- PB4 (MISO)
          (AIN0)    PD6 -|           |- PB3 (MOSI/OC2)
          (AIN1)    PD7 -|           |- PB2 (SS/OC1B)  - LOAD2
 SENSOR - (ICP1)    PB0 -|___________|- PB1 (OC1A)     - LOAD1
*/
/**********************************************************/



/*****************************
Parameters can be changed:
******************************/
//#define DEVICE         (__AVR_ATmega8__)
#define DEVICE         (__AVR_ATtiny13A__)

#define DDR_LOAD       DDRB  //DDR for heating pins
#define PORT_LOAD      PORTB //PORT for heating pins
#define PIN_SENS       PINB  //PIN for sensor
#define DDR_SENS       DDRB  //DDR for sensor
#define PORT_SENS      PORTB //PORT for sensor
#define DDR_LED        DDRB  //DDR for LED
#define PORT_LED       PORTB //PORT for LED

#if defined (__AVR_ATtiny13__) || (__AVR_ATtiny13A__)
#define F_CPU         9600000L //max internal RC freq
#define IO_SENSOR     2     //pin for DS18b20 temperature sensor
#define IO_LED        1     //pin for LED indicator (optional)
#define IO_LOAD1      3     //output #1 for heating relay
#define IO_LOAD2      4     //output #2 for heating relay
#else //defined (__AVR_ATmega8__)
#define F_CPU         8000000L //max internal RC freq
#define IO_SENSOR     0     //pin for DS18b20 temperature sensor
#define IO_LED        5     //pin for LED indicator (optional)
#define IO_LOAD1      1     //output #1 for heating relay
#define IO_LOAD2      2     //output #2 for heating relay
#endif

//#define ACT_IF_T_ABOVE_THRESH           //IO_LOAD1 and IO_LOAD2 will be driven high if temperature ABOVE the thresholds
//#define SENS_SKIP_RESERVED_BYTES_CHECK  //disabling a scratchpad's 7th byte checking


/*****************************
Strictly not recommended to change:
******************************/
#define HYSTERESIS                8 //Celsius degrees, used only for H_REG -> H_PER pass 
#define PULSES_NUM                4 //heating cycles between measurements
#define PULSE_LENGTH              8 //duration of each pulse (seconds)

/*****************************
End of parameters to change
******************************/



#include <avr/io.h>
#include <avr/interrupt.h>



/*****************************
  Macro
******************************/
#if defined (__AVR_ATtiny13__) || (__AVR_ATtiny13A__)
#define TIMER0_OVF_vect		TIM0_OVF_vect
#define EEPROM_RDY        EEPE //EEPE bit in EECR (DS 5.5.3)
#define TICKS_FOR_M_SEC   0x25
#define SECOND_H          0x92 //+1 to drop SECOND_L
//#define SECOND_L        0x7C
  #if !defined (EEPE)
  #define EEPE            1
  #endif
#else //defined (__AVR_ATmega8__)
#define EEPROM_RDY        EEWE //EEWE bit in EECR (DS 8.6.3)
#define TICKS_FOR_M_SEC   0x20
#define SECOND_H          0x7A
//#define SECOND_L        0x12
#endif

#define SENS_RESOLUTION   0x00 //Thermometer resolution (9 bits)
#define SENS_NOID         0xCC //skip identification
#define SENS_CONVERT      0x44 //convert temperature to HEX
#define SENS_RD           0xBE //send all data to master
#define SENS_WR           0x4E //receive data for scratchpad

#define SENS_SET          DDR_SENS   |= 1<<IO_SENSOR
#define SENS_CLR          DDR_SENS   &= ~(1<<IO_SENSOR)
#define LED_ON            PORT_LED   |= 1<<IO_LED
#define LED_OFF           PORT_LED   &= ~(1<<IO_LED)
#define LED_INV           PORT_LED    ^= 1<<IO_LED
#define LOAD1_ON          PORT_LOAD   |= 1<<IO_LOAD1
#define LOAD1_OFF         PORT_LOAD   &= ~(1<<IO_LOAD1)
#define LOAD2_ON          PORT_LOAD   |= 1<<IO_LOAD2
#define LOAD2_OFF         PORT_LOAD   &= ~(1<<IO_LOAD2)

#if defined (__AVR_ATtiny13__) || (__AVR_ATtiny13A__)
#define ALL_LOAD_ON       PORT_LOAD |= ((1<<IO_LOAD1) | (1<<IO_LOAD2))
#define ALL_LOAD_OFF      PORT_LOAD &= ~((1<<IO_LOAD1) | (1<<IO_LOAD2))
#else //defined (__AVR_ATmega8__)
  #warning Be sure that IO_LOAD1 and IO_LOAD2 are on the same PORTx otherwise change this macro.
#define ALL_LOAD_ON       PORT_LOAD |= ((1<<IO_LOAD1) | (1<<IO_LOAD2))
#define ALL_LOAD_OFF      PORT_LOAD &= ~((1<<IO_LOAD1) | (1<<IO_LOAD2))
#endif

#define T_NOSENS          -127
#define T_CRC             -126
#define T_ERR             -120

#define OPERATING_CYCLE   ((PULSES_NUM * PULSE_LENGTH) + PULSE_LENGTH) //measurement period in seconds

#if (OPERATING_CYCLE > 0x7F)
  #error OPERATING_CYCLE will be used as 'int8_t', it should be less than 0x7F.
#endif

enum HEATING_STATE
{
  H_OFF       = 0, //correct sensor data, heating is switched off
  H_REG       = 1, //correct sensor data, heating is switched on in regular mode
  H_PER       = 2, //correct sensor data, heating is switched on in periodic mode
  H_ERR_OFF   = 3, //incorrect or missing sensor data, heating is switched off
  H_ERR_ON    = 4, //incorrect or missing sensor data, heating is switched on for a STARTUP_HEATING_CYCLES
};

enum HEATING_CFG
{
  THRESH_REG       = 0, //index of the regular threshold value (got from EEPROM)
  THRESH_PER       = 1, //index of the periodic threshold value  (got from EEPROM)
  RUNMODE          = 3, //index of the mode value  (got from EEPROM)
  THRESH_STEP      = 4, //temperature step index for MODE_PULSE (calculated)
  PER_PULSES       = 5, //pulse number index (calculated)
  CFG_NUM
};

enum HEATING_MODE
{
  MODE_ALTERNATE   = 0, //if t < THRESH_PER: LOAD1_ON, LOAD2_OFF
                        //if t < THRESH_REG: LOAD1_OFF, LOAD2_ON
  
  MODE_LINEAR      = 1, //if t < THRESH_PER: LOAD1_ON, LOAD2_OFF
                        //if t < THRESH_REG: LOAD1_ON, LOAD2_ON

  MODE_PULSE            //if t < THRESH_PER: LOAD1 and LOAD2 are high for some pulses number (depends on temperature)
                        //if t < THRESH_REG: LOAD1_ON and LOAD2_ON 
};

#define EEP_CFG_REG_ADDR    THRESH_REG //EEPROM addres of the regular threshold value
#define EEP_CFG_PER_ADDR    THRESH_PER //EEPROM addres of the periodic threshold value
#define EEP_CFG_MODE_ADDR   RUNMODE //EEPROM addres of the mode value



#define _DEBUG

#if defined (_DEBUG) && (defined (__AVR_ATtiny13__) || defined (__AVR_ATtiny13A__))
  #undef _DEBUG
#endif



/*****************************
  Functions Declaration
******************************/
void init(void);
static inline void sleep(void);
void wait_us(uint8_t count);
void wait_ms(uint8_t count);
void wait_s(uint8_t count);
uint8_t eepRd(const uint8_t addr);
void readConfig(int8_t * threshold);
uint8_t sensConfig(void);
int8_t measurement(const uint8_t resolution);
void swState(const int8_t t, const int8_t * thrd);

static void sensWr(const uint8_t byte);
static uint8_t sensRd(void);
static uint8_t sensReset(void);
#if defined (_DEBUG)
static void initUart(void);
static void uartWr(const uint8_t byte);
static void logg(const uint8_t * msg);
static uint8_t * ttoa(int8_t val);
#endif



/*****************************
  Global variables
******************************/
volatile uint8_t TCNT = 0; //timer overflow



/*****************************
  Interrupts
******************************/
ISR(TIMER0_OVF_vect)
{
  ++TCNT;
  MCUCR &= ~(1 << SE); //disable sleep mode
}



/*****************************
  Main function
******************************/
void main(void)
{
  uint8_t resolution = 0; //ds18b20 measurement resolution
  int8_t config[CFG_NUM]; //main configuration variables

  init();
  sei();

  LED_ON;

  readConfig(config); //EEPROM reading for thresholds and running mode

  while(1)
  {
    if(!resolution) resolution = sensConfig(); //trying to set the minimum resolution
    swState(measurement(resolution), config);
  }
}

/*****************************
  Public functions
******************************/

void init(void)
{
  ACSR |= 1<<ACD;    //Disable ADC

  #if defined (__AVR_ATtiny13__) || (__AVR_ATtiny13A__)

  PORTB &= ~( (1 << IO_LOAD1) |
        (1 << IO_LOAD2) |
        (1 << IO_SENSOR) |
        (1 << IO_LED) );
  DDRB &= ~(1 << IO_SENSOR);
  DDRB |= ( (1 << IO_LOAD1) | (1 << IO_LOAD2) | (1 << IO_LED) );

  TCCR0B = 0x04; //256 prescaler
  TIMSK0 |= 1<<TOIE0; //Enable TIM0 interrupts

  MCUCR &= ~((1<<SM0) | (1<<SM1)); //idle mode

  #else //(__AVR_ATmega8__)

  PORT_LOAD &= ~( (1<<IO_LOAD1) | (1<<IO_LOAD2) ); //clear PORT for heating pins
  DDR_LOAD |= ( (1<<IO_LOAD1) | (1<<IO_LOAD2) ); //set DDR to output mode

  //sensor pin init
  PORT_SENS &= ~(1<<IO_SENSOR); //clear PORT for sensor
  DDR_SENS &= ~(1<<IO_SENSOR); //clear DDR for sensor

  //LED pin init
  PORT_LED &= ~(1<<IO_LED); //clear PORT for LED;
  DDR_LED |= (1<<IO_LED); //set DDR to output mode

  TCCR0 = 0x04; //256 prescaler
  TIMSK |= 1<<TOIE0; //Enable TIM0 interrupts

  MCUCR &= ~((1<<SM0) | (1<<SM1) | (1<<SM2)); //idle mode

  #if defined (_DEBUG)
    initUart();
  #endif

  #endif
}
/**********************************************************/
static inline void sleep(void)
{
  MCUCR |= 1 << SE;
  asm volatile("sleep"::);
}
/**********************************************************/
void wait_us(uint8_t count)
{
  while(count--)
  {
#if defined (__AVR_ATtiny13__) || (__AVR_ATtiny13A__)
    asm volatile(
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      ::); //x5 for 9.6MHz
#else //(__AVR_ATmega8__)
    asm volatile(
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      "nop\n\t"
      ::); //x4 for 8MHz
#endif    
  }
}
/**********************************************************/
void wait_ms(uint8_t count)
{
  while(count--)
  {
    uint8_t i = 4;
    while(i--) wait_us(250); 
  }
}
/**********************************************************/
void wait_s(uint8_t count)
{
  volatile uint8_t timer = TCNT;

  while(count--)
  {
    timer += SECOND_H;
    while(TCNT != timer)
    {
      sleep();
    }
  }
}
/**********************************************************/
uint8_t eepRd(const uint8_t addr)
{
  while(EECR & (1 << EEPROM_RDY)); //EEPROM_RDY - macro
  EEARL = addr;
  EECR |= (1 << EERE);
  return EEDR;
}
/**********************************************************/
void readConfig(int8_t * cfg)
{
  cfg[THRESH_REG] = (int8_t)eepRd(EEP_CFG_REG_ADDR);
  cfg[THRESH_PER] = (int8_t)eepRd(EEP_CFG_PER_ADDR);
  cfg[RUNMODE] = (int8_t)eepRd(EEP_CFG_MODE_ADDR);

  #if !defined (ACT_IF_T_ABOVE_THRESH)
  if(cfg[THRESH_PER] < cfg[THRESH_REG]) cfg[THRESH_PER] = cfg[THRESH_REG];
  #else
  if(cfg[THRESH_PER] > cfg[THRESH_REG]) cfg[THRESH_PER] = cfg[THRESH_REG];
  #endif

  cfg[THRESH_STEP] = (cfg[THRESH_REG] - cfg[THRESH_PER]);
  if(cfg[THRESH_STEP] < 0) cfg[THRESH_STEP] = 0 - cfg[THRESH_STEP];  

  if(cfg[THRESH_STEP] >= PULSES_NUM)
  {
    cfg[PER_PULSES] = PULSES_NUM;
    cfg[THRESH_STEP] = ((uint8_t)cfg[THRESH_STEP] / PULSES_NUM);
  }
  else
  {
    cfg[PER_PULSES] = cfg[THRESH_STEP];
    cfg[THRESH_STEP] = 1;
  }
}
/**********************************************************/
uint8_t sensConfig(void)
{
  if(sensReset()) return 0;
  sensWr(SENS_NOID);
  sensWr(SENS_WR);
  sensWr(T_ERR); //TH byte
  sensWr(0xFF); //TL byte
  sensWr(SENS_RESOLUTION); //Config byte
  return 1;
}
/**********************************************************/
int8_t measurement(const uint8_t resolution)
{
  //uint8_t fractPart = 0; //fractional part of the temperature

  if(sensReset())
    return T_NOSENS;
  else
  {
    uint8_t bytes = 0;  //bytes counter
    uint8_t crc = 0;    //crc result
    uint8_t data = 0;   //received data byte
    int8_t intPart = 0; //integer part of the temperature

    sensWr(SENS_NOID);
    sensWr(SENS_CONVERT);

    if(resolution)
      wait_ms(100);
    else
      wait_s(1);

    if(sensReset())
      return T_NOSENS;

    sensWr(SENS_NOID);
    sensWr(SENS_RD);

    while(bytes < 8)
    {
      data = sensRd();

      if(!bytes) intPart |= (data >> 4); //fractPart = data & 0x0F; //not used
      else if(bytes == 1) intPart |= (data << 4);
#if !defined (SENS_SKIP_RESERVED_BYTES_CHECK)
      else if(bytes == 5 && data != 0xFF) return T_NOSENS;
      else if(bytes == 7 && data != 0x10) return T_NOSENS;
#endif

      ++bytes;

      for(uint8_t i=0; i<8; ++i)
      {
        if((crc ^ data) & 0x01)
        {
          crc >>= 1;
          crc ^= 0x8C;
        }
        else crc >>= 1;

        data >>= 1;
      }

    }
    data = sensRd();

    if(data != crc) return T_CRC;
    return intPart;
  }

}
/**********************************************************/
void swState(const int8_t t, const int8_t * cfg)//, enum HEATING_STATE st)
{
  static enum HEATING_STATE state = H_OFF;
  uint8_t pulses = 0;
  int8_t temp;

  //switching the system state:
  if(t <= T_ERR) state = H_ERR_OFF; //no sensor, bad temperature or CRC
#if !defined (ACT_IF_T_ABOVE_THRESH)
  else if(t <= cfg[THRESH_REG]) state = H_REG;
  else
  {
    if(state != H_REG && t <= cfg[THRESH_PER]) state = H_PER;
    else
    {
      if(t >= (cfg[THRESH_REG] + HYSTERESIS)) state = H_PER;
      if(t >= (cfg[THRESH_PER])) state = H_OFF;
    }

    if(state == H_PER)
    {
      temp = cfg[THRESH_PER] - cfg[THRESH_STEP]; //starting value for determining an amount of steps for H_PER
      while(++pulses < cfg[PER_PULSES]) //determine a stage of periodic heating
      {
        if(t > temp) break;
        temp -= cfg[THRESH_STEP];
      }

      pulses *= PULSE_LENGTH;
    }
  }
#else
  else if(t >= cfg[THRESH_REG]) state = H_REG;
  else
  {
    if(state != H_REG && t >= cfg[THRESH_PER]) state = H_PER;
    else
    {
      if(t <= (cfg[THRESH_REG] - HYSTERESIS)) state = H_PER;
      if(t <= (cfg[THRESH_PER])) state = H_OFF;
    }

    if(state == H_PER)
    {
      temp = cfg[THRESH_PER] + cfg[THRESH_STEP]; //starting value for determining an amount of steps for H_PER
      while(++pulses < cfg[PER_PULSES]) //determine a stage of periodic heating
      {
        if(t < temp) break;
        temp += cfg[THRESH_STEP];
      }

      pulses *= PULSE_LENGTH;
    }
  }
#endif

  //switching the load outputs:
  if(cfg[RUNMODE] == MODE_ALTERNATE || cfg[RUNMODE] == MODE_LINEAR)
  {
    if(state == H_PER) //the same state for both modes
    {
      LOAD1_ON;
      LOAD2_OFF;
    }
    else if(state == H_REG)
    {
      if(cfg[RUNMODE] == MODE_ALTERNATE) LOAD1_OFF;
      else if(cfg[RUNMODE] == MODE_LINEAR) LOAD1_ON;
      LOAD2_ON;
    }
    else ALL_LOAD_OFF;
  }
  else //MODE_PULSE
  {
    if(state == H_REG) ALL_LOAD_ON; //heating outputs work together
    else if(state != H_PER) ALL_LOAD_OFF;
  }


  temp = OPERATING_CYCLE; //number of seconds until the next measurement
  

  //processing and indicting during operating cycle:
  while(temp--)
  {
    wait_s(1);

    if(state >= H_ERR_OFF) //any error case
    {
      //1 short blink followed by a continuous light:
      LED_ON;
      wait_ms(30);
      LED_OFF;
      wait_ms(250);
      LED_ON;
      return; //leaves the cycle after 1.28 sec without waiting for a full OPERATING_CYCLE
    }
    else if(state == H_PER)
    {
      if(cfg[RUNMODE] >= MODE_PULSE)
      {
        if(pulses)
        {
          ALL_LOAD_ON; //both heating outputs
          --pulses;
        }
        else
        {
          ALL_LOAD_OFF; //both heating outputs
        }
      }

      LED_INV;
    }
    else
    {
      LED_ON; //for H_REG
      if(state == H_OFF) //one short blink every second
      {
        wait_ms(30);
        LED_OFF;
      }
    }
  } //while(temp)
}



/*****************************
  Static functions
******************************/

/**********************************************************/
static void sensWr(const uint8_t byte)
{
  asm volatile("cli"::);
  for(uint8_t i=0; i<8; ++i)
  {
    SENS_SET;
    wait_us(5);
    if(byte & (1 << i))
      SENS_CLR;
    wait_us(65);
    SENS_CLR;
    wait_us(5);
  }
  asm volatile("sei"::);
}
/**********************************************************/
static uint8_t sensRd(void)
{
  uint8_t data = 0;

  asm volatile("cli"::);
  for(uint8_t i=0; i<8; ++i)
  {
    SENS_SET;
    wait_us(3);
    SENS_CLR;
    wait_us(15);
    data >>= 1;
    if(PIN_SENS & (1<<IO_SENSOR))
      data |= 0x80;
    wait_us(45);
  }
  asm volatile("sei"::);

  return data;
}
/**********************************************************/
static uint8_t sensReset(void)
{
  uint8_t pdhigh = 1; //Presence-Detect is high? it's low if sensor is found

  cli();
  
  SENS_SET;
  wait_ms(1);
  SENS_CLR;
  wait_us(65);
  pdhigh = (PIN_SENS & (1<<IO_SENSOR));

  sei();

  if(!pdhigh) //success
    wait_us(255);

  return pdhigh; //not found
}
/**********************************************************/


#if defined _DEBUG
/*****************************
  Debug functions
******************************/

static void initUart(void)
{
  //UBRRH = 0;  //UBBR = f/(16*band)-1; f=8000000 Hz, band=9600
  UBRRL = 51; //Normal dual asynchronous mode

  UCSRA = 0x00;
    //  RXC     - receive ending
    //  |TXC    - transmit ending
    //  ||UDRE    - no data to transmit
    //  |||FE   - frame error
    //  ||||DOR   - buffer overflow error
    //  |||||PE   - parity error
    //  ||||||U2X - dual speed
    //  |||||||MPCM - multiprocessor mode
    //  76543210
    //  00000000  - UCSRA
  UCSRB |= (1<<TXEN);
    //  RXCIE   - receive data interrupt
    //  |TXCIE    - interrupt after transmitting data
    //  ||UDRIE   - interrupt after no data to transmit
    //  |||RXEN   - enable receive
    //  ||||TXEN  - enable transmit
    //  |||||UCSZ2  - UCSZ0..2 size of data frame
    //  ||||||RXB8  - 9 bits of received data
    //  |||||||TXB8 - 9 bits of transmitted data
    //  76543210
    //  00011000  - UCSRB

  UCSRC |= ( (1<<URSEL) | (1<<UCSZ0) | (1<<UCSZ1) );
    //  URSEL   - always 1
    //  |UMSEL    - 0 - asynchronous, 1 - synchronous
    //  ||UPM1    - UPM0..1 - parity
    //  |||UPM0   - UPM0..1 - parity
    //  ||||USBS  - stop-bits: 0 - 1, 1 - 2
    //  |||||UCSZ1  - UCSZ0..2 size of data frame
    //  ||||||UCSZ0 - UCSZ0..2 size of data frame
    //  |||||||UCPOL- clocking only for synchronous mode
    //  76543210
    //  10000110  - UCSRC
}

static void uartWr(const uint8_t byte)
{
  while ( !(UCSRA & (1<<UDRE)) ) {} //waiting while transmit buffer if not empty
  UDR = byte;
}

static void logg(const uint8_t * msg)
{
  while(*(msg))
  {
    uartWr(*(msg++));
  }
}

static uint8_t * itoh(int8_t val)
{
  
}

static uint8_t * ttoa(int8_t val)
{
  static const uint8_t zeroChar = 48;
  static uint8_t num[5];
  uint8_t decimals = 0;
  uint8_t digit = 0;

  num[1] = 0; //the first character is skipped
  num[2] = 0;
  num[3] = 0;
  num[4] = 0;

  if(val < 0)
  {
    val = 0 - val;
    num[digit] = 45; //'-'
    ++digit;
  }

  if(val >= 100)
  {
    val -= 100;
    num[digit] = 1 + zeroChar; //'1'
    if(val >= 100)
    {
      num[digit] += 1;
      val -= 100;
    }
    ++digit;
  
    if(!val) //value = 100
    {
      num[digit] = zeroChar; //'0'
      num[++digit] = zeroChar; //'0'

      return num;
    }
    else if(val < 10) //value = 101..109
    {
      num[digit] = zeroChar; //'0'
      num[++digit] = val + zeroChar; //ascii char of number
      
      return num;
    }
  }

  while(val >= 10)
  {
    val -= 10;
    ++decimals;
  }

  if(decimals)
  {
    num[digit] = decimals + zeroChar; //ascii char of number
    ++digit;
  }

  num[digit] = val + zeroChar; //ascii char of number
      
  return num;
}
#endif


