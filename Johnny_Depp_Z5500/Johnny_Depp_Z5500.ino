/*
   New brains for the deceived Logitec Z550 that also goes by
   the name of Johnny Depp
*/

//===================================
// includes
//===================================

#include <EEPROM.h>
#include <string.h>


//===================================
// definitions
//===================================

// Debugging
#define SERIAL_ENABLE           (0)

// SPI pins
#define SPI_CS     10
#define SPI_SDA    11
#define SPI_SCL    12

// I2C pins
#define I2C_SDA     5
#define I2C_SCL     4
#define I2C_DELAY   5
// Delay of 5 usec ~= 100kHz I2C
#define I2C_OK      0
#define I2C_ERROR   1

//BOARD	                    DIGITAL PINS USABLE FOR INTERRUPTS
//Arduino Nano (328-based):    2, 3
// rotary encoder
#define ENCPINA 3
#define ENCPINB 2

// HW power/mute control (WA13, WA16 = ON, WA14 = MUTE)
#define PIN_ON    6
#define PIN_MUTE  7

// Buttons
#define BUTTON_EFFECT   A0
#define BUTTON_SETTING  A1
#define BUTTON_LEVEL    A4
#define BUTTON_MUTE     A5

// NJW1150 control (via I2C)
#define NJW1150_ADDR          0x88

#define NJW1150_REG_VOLUME    0x00
#define NJW1150_REG_LEFT      0x01
#define NJW1150_REG_RIGHT     0x02
#define NJW1150_REG_CENTER    0x03
#define NJW1150_REG_SL        0x04
#define NJW1150_REG_SR        0x05
#define NJW1150_REG_SUB       0x06
#define NJW1150_REG_TONE      0x07
#define NJW1150_REG_MUTE      0x08

// LCD pannel (via SPI)
#define LCD_BUFFER_SIZE         (10*8)
#define LCD_LINE_SIZE           (LCD_BUFFER_SIZE / 2)
#define LCD_LINE_VISIBLE_LENGTH (20)

// Levels
#define MAX_VOLUME              (32)
#define MIN_SUB                 (-9)
#define MAX_SUB                 (6)
#define MIN_BALANCE             (-7)
#define MAX_BALANCE             (7)
#define MIN_CENTER              (-9)
#define MAX_CENTER              (6)
#define MIN_SURROUND            (-9)
#define MAX_SURROUND            (6)

#define LEVEL_STATE_VOLUME      (0)
#define LEVEL_STATE_SUB         (1)
#define LEVEL_STATE_BALANCE     (2)
#define LEVEL_STATE_CENTER      (3)
#define LEVEL_STATE_SURROUND    (4)
#define LEVEL_STATE_MAX         (LEVEL_STATE_SURROUND)

// Timing
#define LOOP_IDLE_PERIOD        (20)
#define INACTIVITY_COUNT_INIT   (150)


//===================================
// type definitions
//===================================

typedef signed char    s8_t;
typedef signed short   s16_t;

typedef unsigned char  u8_t;
typedef unsigned short u16_t;

union LcdBuf_t
{
   char buf[LCD_BUFFER_SIZE];
   struct
   {
      char line0[LCD_LINE_SIZE];
      char line1[LCD_LINE_SIZE];
   }lin;
};

typedef union
{
   u8_t data[6];
   struct
   {
      s8_t     volume;
      u8_t     mute;
      s8_t     sub;
      s8_t     balance;
      s8_t     center;
      s8_t     surround;
   }fields;
}settings_t;


//===================================
// global variables
//===================================

char  msg[LCD_LINE_SIZE];
#if SERIAL_ENABLE
char  ser_msg[128];
#endif

LcdBuf_t lcd_buffer;

volatile byte seqA = 0;
volatile byte seqB = 0;
volatile byte cnt1 = 0;
volatile byte cnt2 = 0;
volatile boolean right  = false;
volatile boolean left   = false;
volatile boolean button = false;

settings_t levels;

u8_t       flash_idx;
u8_t       flash_idx_verify;
settings_t flash_levels;
settings_t flash_levels_verify;


bool     update_screen  = false;
bool     update_njw1150 = false;

bool     button_effect_val;
bool     button_setting_val;
bool     button_level_val;
bool     button_mute_val;

bool     button_effect_pressed  = false;
bool     button_setting_pressed = false;
bool     button_level_pressed   = false;
bool     button_mute_pressed    = false;

u8_t     level_state = LEVEL_STATE_VOLUME;
u8_t     inactivity_countdown;


//===================================
// SPI functions
//===================================

void spi_init(void)
{
   pinMode(SPI_SCL, OUTPUT);
   digitalWrite(SPI_SCL, LOW);

   pinMode(SPI_CS, OUTPUT);
   digitalWrite(SPI_CS, LOW);

   pinMode(SPI_SDA, OUTPUT);
   digitalWrite(SPI_SDA, LOW);
}

void spi_write(u8_t val)
{
   u8_t k;
   bool b;

   digitalWrite(SPI_CS, LOW);
   delayMicroseconds(2);
   k = 8;
   while(k--)
   {
      b = (val & 0x80) ? HIGH : LOW;
      val <<= 1;

      digitalWrite(SPI_SDA, b);
      digitalWrite(SPI_SCL, LOW);
      delayMicroseconds(2);
      digitalWrite(SPI_SCL, HIGH);
   }
   digitalWrite(SPI_CS,  HIGH);
   delayMicroseconds(60);
}


//===================================
// I2C functions
//===================================

void i2c_start (void)
{
   u8_t timeout = 255;

   while( !digitalRead(I2C_SCL) &&
          !digitalRead(I2C_SDA) &&
          timeout-- )
   {
      delayMicroseconds(2);
   }

   pinMode(I2C_SDA, OUTPUT);
   digitalWrite(I2C_SDA, LOW);
   delayMicroseconds(I2C_DELAY);
   pinMode(I2C_SCL, OUTPUT);
   digitalWrite(I2C_SCL, LOW);
   delayMicroseconds(I2C_DELAY);
}


void i2c_stop (void)
{
   u8_t timeout = 255;

   pinMode(I2C_SDA, OUTPUT);
   digitalWrite(I2C_SDA, LOW);
   delayMicroseconds(I2C_DELAY);
   digitalWrite(I2C_SCL, HIGH);
   while( !digitalRead(I2C_SCL) &&
          timeout-- )

   delayMicroseconds(I2C_DELAY);
   digitalWrite(I2C_SDA, HIGH);
   delayMicroseconds(I2C_DELAY);
}


boolean i2c_write_byte(u8_t val)
{
   u8_t k;
   bool b;
   u8_t timeout = 255;

   // write byte
   k = 8;
   while(k--)
   {
      b = (val & 0x80) ? HIGH : LOW;
      val <<= 1;

      pinMode(I2C_SDA, OUTPUT);
      digitalWrite(I2C_SDA, b);
      digitalWrite(I2C_SCL, HIGH);
      delayMicroseconds(I2C_DELAY);
      pinMode(I2C_SCL, OUTPUT);
      digitalWrite(I2C_SCL, LOW);
      delayMicroseconds(I2C_DELAY);
   }

   // check ack
   pinMode(I2C_SDA, INPUT);
   digitalWrite(I2C_SDA, HIGH);
   pinMode(I2C_SCL, INPUT);
   digitalWrite(I2C_SCL,  HIGH);
   while( !digitalRead(I2C_SCL) &&
          timeout-- )

   delayMicroseconds(I2C_DELAY);
   b = digitalRead(I2C_SDA);
   pinMode(I2C_SCL, OUTPUT);
   digitalWrite(I2C_SCL, LOW);
   delayMicroseconds(I2C_DELAY);

   return !b;
}


u8_t i2c_read_byte(bool ack)
{
   u8_t k;
   u8_t B = 0;
   bool b;
   u8_t timeout = 255;

   // read byte
   k = 8;
   pinMode(I2C_SDA, INPUT);
   digitalWrite(I2C_SDA, HIGH);
   while(k--)
   {
      pinMode(I2C_SCL, INPUT);
      digitalWrite(I2C_SCL, HIGH);
      while(!digitalRead(I2C_SCL));

      delayMicroseconds(I2C_DELAY);
      b = digitalRead(I2C_SDA);
      pinMode(I2C_SCL, OUTPUT);
      digitalWrite(I2C_SCL, LOW);
      delayMicroseconds(I2C_DELAY);
      B |= (b == true) ? 0x01 : 0x00;
      B <<= 1;
   }

   // ack (or nack)
   if (!ack)
   {
      pinMode(I2C_SDA, OUTPUT);
      digitalWrite(I2C_SDA, LOW);
   }
   pinMode(I2C_SCL, INPUT);
   digitalWrite(I2C_SCL,  HIGH);
   while( !digitalRead(I2C_SCL) &&
          timeout-- )

   delayMicroseconds(I2C_DELAY);
   pinMode(I2C_SCL, OUTPUT);
   digitalWrite(I2C_SCL, LOW);
   delayMicroseconds(I2C_DELAY);

   return B;
}


u8_t i2c_read_register(u8_t addr, u8_t subaddr, u8_t *data)
{
   bool ack;

   if (!data)
   {
      return I2C_ERROR;
   }

   // start
   i2c_start();

   // addr
   ack = i2c_write_byte(addr);
   if (!ack)
   {
      // no ack from slave
      return I2C_ERROR;
   }

   // subaddr
   ack = i2c_write_byte(subaddr);
   if (!ack)
   {
      // no ack from slave
      return I2C_ERROR;
   }

   *data = i2c_read_byte(false); // nack to signal the last byte

   // release bus
   i2c_stop();

   return I2C_OK;
}


u8_t i2c_write_register(u8_t addr, u8_t subaddr, u8_t data)
{
   bool ack;

   // start
   i2c_start();

   // addr
   ack = i2c_write_byte(addr);
   if (!ack)
   {
      // no ack from slave
      return I2C_ERROR;
   }

   // subaddr
   ack = i2c_write_byte(subaddr);
   if (!ack)
   {
      // no ack from slave
      return I2C_ERROR;
   }

   // data
   ack = i2c_write_byte(data);
   if (!ack)
   {
      // no ack from slave
      return I2C_ERROR;
   }

   // release bus
   i2c_stop();

   return I2C_OK;
}


void i2c_init (void)
{
   pinMode(I2C_SCL, OUTPUT);
   pinMode(I2C_SDA, OUTPUT);

   i2c_stop();
}




//===================================
// LCD functions
//===================================

void lcd_clear_buffer(void)
{
   for(int k = 0; k < LCD_BUFFER_SIZE; k++)
   {
      lcd_buffer.buf[k] = 0x20; // space
   }
}

void lcd_cls(void)
{
  spi_write(0x38);
  spi_write(0x3C);

//  spi_write(0x38); // '38' switch to command register??
  spi_write(0x07);

  spi_write(0x38); // '38' switch to command register??
  spi_write(0x0C); // Display on, cursor off

  spi_write(0x38); // '38' switch to command register??
  spi_write(0x01); // clear display

  spi_write(0x38); // '38' switch to command register??
  spi_write(0x02); // cursor to first position
  delayMicroseconds(50);
}

void lcd_init(void)
{
  lcd_cls();
  lcd_clear_buffer();
}

void lcd_write_char(char symb)
{
   spi_write(0x80); // 0x80 0+1 characters are following
   spi_write(symb);
   delayMicroseconds(50);
}

void lcd_update_display(void)
{
   int pos = 0;
   int m;

   for (m = 20; m < LCD_LINE_SIZE; m++)
   {
      lcd_buffer.lin.line0[m] = ' ';
      lcd_buffer.lin.line1[m] = ' ';
   }
   for (m = 0; m < 10; m++)
   {
      spi_write(0x3C); // '3C' switch to data register?
      for(int k = 0; k < 8; k++)
      {
        lcd_write_char(lcd_buffer.buf[pos + k]);
      }
      pos += 8;
   }
}


//===================================
// Rotary functions
//===================================

void rotary_update (void)
{
#if SERIAL_ENABLE
   //Serial.println("interrupt");
#endif

   // Read A and B signals
   boolean A_val = digitalRead(ENCPINA);
   boolean B_val = digitalRead(ENCPINB);

   // Record the A and B signals in separate sequences
   seqA <<= 1;
   seqA |= A_val;

   seqB <<= 1;
   seqB |= B_val;

   // Mask the MSB four bits
   seqA &= 0b00001111;
   seqB &= 0b00001111;

   // Compare the recorded sequence with the expected sequence
   if (seqA == 0b00001001 && seqB == 0b00000011)
   {
      cnt1++;
      left = true;
   }

   if (seqA == 0b00000011 && seqB == 0b00001001)
   {
      cnt2++;
      right = true;
   }
}

void rotary_init(void)
{
   // Attach update function to interrupt events
   pinMode(ENCPINA, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(ENCPINA), rotary_update, CHANGE);

   pinMode(ENCPINB, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(ENCPINB), rotary_update, CHANGE);
}


//===================================
// NJW1150 functions
//===================================

void njw1150_init(void)
{
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_VOLUME, 0x20);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_LEFT,   0x00);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_RIGHT,  0x00);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_CENTER, 0x03);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_SUB,    0x0C);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_MUTE,   0x3F);
}


void njw1150_set_volume(void)
{
   u8_t val = 0;

   if ( (levels.fields.volume == 0) ||
         levels.fields.mute )
   {
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_MUTE,   0x3F);
      digitalWrite(PIN_MUTE, HIGH);
      val = 0x50; // mute
   }
   else
   {
      digitalWrite(PIN_MUTE, LOW);
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_MUTE,   0x00);
      if (levels.fields.volume >= 32)
      {
         val = 0;    // 0dB attenuation
      }
      else if (levels.fields.volume > 16)
      {
         val = 32 - levels.fields.volume;
      }
      else
      {
         val = 28 - levels.fields.volume;
         val <<= 1;
      }
   }
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_VOLUME, val);
}


void njw1150_set_sub(void)
{
   u8_t val = 0;

   val = (u8_t) (MAX_SUB - levels.fields.sub);
   if (val > 20)
   {
      val = 20;
   }
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_SUB, val);
}


void njw1150_set_balance(void)
{
   u8_t val = 0;

   if (levels.fields.balance == 0)
   {
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_LEFT,  0);
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_RIGHT, 0);
   }
   else if (levels.fields.balance > 0)
   {
      // "right" means attenuate left
      val = levels.fields.balance;
      val <<= 2;
      if (val > 0x1F)
      {
         val = 0x1F;
      }
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_LEFT,  val);
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_RIGHT, 0);
   }
   else
   {
      // "left" means attenuate right
      val = (u8_t) (-levels.fields.balance);
      val <<= 2;
      if (val > 0x1F)
      {
         val = 0x1F;
      }
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_LEFT,  0);
      i2c_write_register(NJW1150_ADDR, NJW1150_REG_RIGHT, val);
   }
}


void njw1150_set_center(void)
{
   u8_t val = 0;

   val = (u8_t) (MAX_CENTER - levels.fields.center);
   if (val > 20)
   {
      val = 20;
   }
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_CENTER, val);
}


void njw1150_set_surround(void)
{
   u8_t val = 0;

   val = (u8_t) (MAX_SURROUND - levels.fields.surround);
   if (val > 20)
   {
      val = 20;
   }
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_SL, val);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_SL, val);
}


//===================================
// Button functions
//===================================

void button_init(void)
{
   pinMode(BUTTON_EFFECT   , INPUT);
   pinMode(BUTTON_SETTING  , INPUT);
   pinMode(BUTTON_LEVEL    , INPUT);
   pinMode(BUTTON_MUTE     , INPUT);
   button_effect_val  = digitalRead(BUTTON_EFFECT  );
   button_setting_val = digitalRead(BUTTON_SETTING );
   button_level_val   = digitalRead(BUTTON_LEVEL   );
   button_mute_val    = digitalRead(BUTTON_MUTE    );
}


void button_task(void)
{
   if (digitalRead(BUTTON_EFFECT) != button_effect_val)
   {
      button_effect_val = !button_effect_val;
      if (!button_effect_val)
      {
         button_effect_pressed = true;
      }
   }

   if (digitalRead(BUTTON_SETTING) != button_setting_val)
   {
      button_setting_val = !button_setting_val;
      if (!button_setting_val)
      {
         button_setting_pressed = true;
      }
   }

   if (digitalRead(BUTTON_LEVEL) != button_level_val)
   {
      button_level_val = !button_level_val;
      if (!button_level_val)
      {
         button_level_pressed = true;
      }
   }

   if (digitalRead(BUTTON_MUTE) != button_mute_val)
   {
      button_mute_val = !button_mute_val;
      if (!button_mute_val)
      {
         button_mute_pressed = true;
      }
   }
}


//===================================
// Level functions
//===================================

void set_volume(void)
{
   u8_t vol_dif2;
   u8_t k;

   vol_dif2 = levels.fields.volume>>1;

   // Update display
#if SERIAL_ENABLE
   Serial.println (levels.fields.volume);
#endif

   lcd_clear_buffer();
   strncpy( &lcd_buffer.lin.line0[4], "Volume: ", 8);
   sprintf( msg, "%02d", levels.fields.volume);
   strncpy( &lcd_buffer.lin.line0[12], msg, 2);
   if (  (levels.fields.volume == 0) ||
         (levels.fields.mute) )
   {
      strncpy( &lcd_buffer.lin.line1[5], "< mute >", 8);
   }
   else
   {
      for(k = 0; k < vol_dif2; k++)
      {
         lcd_buffer.lin.line1[k+2] = '=';
#if SERIAL_ENABLE
         Serial.print ('X');
#endif
      }
      if (levels.fields.volume & 1)
      {
         lcd_buffer.lin.line1[k+2] = '-';
#if SERIAL_ENABLE
         Serial.print ('x');
#endif
      }
#if SERIAL_ENABLE
   Serial.print ("\n");
#endif
   }

   update_screen  = true;
   update_njw1150 = true;
}

void set_sub(void)
{
   s8_t k;

   lcd_clear_buffer();
   if ( levels.fields.sub == 0 )
   {
      strncpy( &lcd_buffer.lin.line0[5], "Sub:   00",  9);
   }
   else if ( levels.fields.sub >= 0 )
   {
      strncpy( &lcd_buffer.lin.line0[5], "Sub:  +",  7);
      sprintf( msg, "%02d", levels.fields.sub);
      strncpy( &lcd_buffer.lin.line0[12], msg, 2);
   }
   else
   {
      strncpy( &lcd_buffer.lin.line0[5], "Sub:  -",  7);
      sprintf( msg, "%02d", -levels.fields.sub);
      strncpy( &lcd_buffer.lin.line0[12], msg, 2);
   }
   for (k = 0; k <= (MAX_SUB - MIN_SUB); k++)
   {
      if (k == (levels.fields.sub - MIN_SUB))
      {
         strncpy( &lcd_buffer.lin.line1[k + 2], "|", 1);
      }
      else
      {
         strncpy( &lcd_buffer.lin.line1[k + 2], "-", 1);
      }
   }
   if (levels.fields.sub)
   {
      strncpy( &lcd_buffer.lin.line1[2-MIN_SUB], "+", 1);
   }


   update_screen  = true;
   update_njw1150 = true;
}


void set_balance(void)
{
   s8_t k;

   lcd_clear_buffer();
   if (levels.fields.balance == 0)
   {
      strncpy( &lcd_buffer.lin.line0[2], "Balance:  00", 12);
      strncpy( &lcd_buffer.lin.line1[2], "-------|-------", 15);
   }
   else
   {
      if ( levels.fields.balance > 0 )
      {
         strncpy( &lcd_buffer.lin.line0[2], "Balance: R", 10);
         sprintf( msg, "%02d", levels.fields.balance);
         strncpy( &lcd_buffer.lin.line0[12], msg, 2);
      }
      else
      {
         strncpy( &lcd_buffer.lin.line0[2], "Balance: L", 10);
         sprintf( msg, "%02d", -levels.fields.balance);
         strncpy( &lcd_buffer.lin.line0[12], msg, 2);
      }
      for (k = 0; k <= (MAX_BALANCE - MIN_BALANCE); k++)
      {
         if (k == (levels.fields.balance - MIN_BALANCE))
         {
            strncpy( &lcd_buffer.lin.line1[k + 2], "|", 1);
         }
         else
         {
            strncpy( &lcd_buffer.lin.line1[k + 2], "-", 1);
         }
      }
      strncpy( &lcd_buffer.lin.line1[2-MIN_BALANCE], "+", 1);
   }

   update_screen  = true;
   update_njw1150 = true;
}


void set_center (void)
{
   s8_t k;

   lcd_clear_buffer();
   if ( levels.fields.center == 0 )
   {
      strncpy( &lcd_buffer.lin.line0[3], "Center:  00",  11);
   }
   else if ( levels.fields.center >= 0 )
   {
      strncpy( &lcd_buffer.lin.line0[3], "Center: +",  9);
      sprintf( msg, "%02d", levels.fields.center);
      strncpy( &lcd_buffer.lin.line0[12], msg, 2);
   }
   else
   {
      strncpy( &lcd_buffer.lin.line0[3], "Center: -",  9);
      sprintf( msg, "%02d", -levels.fields.center);
      strncpy( &lcd_buffer.lin.line0[12], msg, 2);
   }
   for (k = 0; k <= (MAX_CENTER - MIN_CENTER); k++)
   {
      if (k == (levels.fields.center - MIN_CENTER))
      {
         strncpy( &lcd_buffer.lin.line1[k + 2], "|", 1);
      }
      else
      {
         strncpy( &lcd_buffer.lin.line1[k + 2], "-", 1);
      }
   }
   if (levels.fields.center)
   {
      strncpy( &lcd_buffer.lin.line1[2-MIN_CENTER], "+", 1);
   }

   update_screen  = true;
   update_njw1150 = true;
}


void set_surround (void)
{
   s8_t k;

   lcd_clear_buffer();
   if ( levels.fields.surround == 0 )
   {
      strncpy( &lcd_buffer.lin.line0[1], "Surround:  00",  13);
   }
   else if ( levels.fields.surround >= 0 )
   {
      strncpy( &lcd_buffer.lin.line0[1], "Surround: +", 11);
      sprintf( msg, "%02d", levels.fields.surround);
      strncpy( &lcd_buffer.lin.line0[12], msg, 2);
   }
   else
   {
      strncpy( &lcd_buffer.lin.line0[1], "Surround: -", 11);
      sprintf( msg, "%02d", -levels.fields.surround);
      strncpy( &lcd_buffer.lin.line0[12], msg, 2);
   }
   for (k = 0; k <= (MAX_SURROUND - MIN_SURROUND); k++)
   {
      if (k == (levels.fields.surround - MIN_SURROUND))
      {
         strncpy( &lcd_buffer.lin.line1[k + 2], "|", 1);
      }
      else
      {
         strncpy( &lcd_buffer.lin.line1[k + 2], "-", 1);
      }
   }
   if (levels.fields.surround)
   {
      strncpy( &lcd_buffer.lin.line1[2-MIN_SURROUND], "+", 1);
   }

   update_screen  = true;
   update_njw1150 = true;
}


void handle_level(void)
{
   // Update level state machine
   if (!inactivity_countdown)
   {
      level_state = LEVEL_STATE_VOLUME;
   }
   else
   {
      if (button_level_pressed)
      {
         level_state++;
         if (level_state > LEVEL_STATE_MAX)
         {
            level_state = LEVEL_STATE_VOLUME;
         }
      }
   }

   if (button_level_pressed)
   {
      button_level_pressed = false;
   }

   if (button_mute_pressed)
   {
      if (levels.fields.mute)
         levels.fields.mute = 0;
      else
         levels.fields.mute = 1;
      button_mute_pressed = false;
   }

   // Update level setting
   switch(level_state)
   {
      case LEVEL_STATE_VOLUME:
      {
         if (right)
         {
            levels.fields.volume += cnt2;
            cnt2   = 0;
            right  = false;
            if (levels.fields.volume > MAX_VOLUME)
               levels.fields.volume = MAX_VOLUME;
         }
         if (left)
         {
            levels.fields.volume -= cnt1;
            cnt1   = 0;
            left   = false;
            if (levels.fields.volume < 0)
               levels.fields.volume = 0;
         }
         set_volume();
         break;
      }
      case LEVEL_STATE_SUB:
      {
         if (right)
         {
            levels.fields.sub += cnt2;
            cnt2   = 0;
            right  = false;
            if (levels.fields.sub > MAX_SUB)
               levels.fields.sub = MAX_SUB;
         }
         if (left)
         {
            levels.fields.sub -= cnt1;
            cnt1   = 0;
            left   = false;
            if (levels.fields.sub < MIN_SUB)
               levels.fields.sub = MIN_SUB;
         }
         set_sub();
         break;
      }
      case LEVEL_STATE_BALANCE:
      {
         if (right)
         {
            levels.fields.balance += cnt2;
            cnt2   = 0;
            right  = false;
            if (levels.fields.balance > MAX_BALANCE)
               levels.fields.balance = MAX_BALANCE;
         }
         if (left)
         {
            levels.fields.balance -= cnt1;
            cnt1   = 0;
            left   = false;
            if (levels.fields.balance < MIN_BALANCE)
               levels.fields.balance = MIN_BALANCE;
         }
         set_balance();
         break;
      }
      case LEVEL_STATE_CENTER:
      {
         if (right)
         {
            levels.fields.center += cnt2;
            cnt2   = 0;
            right  = false;
            if (levels.fields.center > MAX_CENTER)
               levels.fields.center = MAX_CENTER;
         }
         if (left)
         {
            levels.fields.center -= cnt1;
            cnt1   = 0;
            left   = false;
            if (levels.fields.center < MIN_CENTER)
               levels.fields.center = MIN_CENTER;
         }
         set_center();
         break;
      }
      case LEVEL_STATE_SURROUND:
      {
         if (right)
         {
            levels.fields.surround += cnt2;
            cnt2   = 0;
            right  = false;
            if (levels.fields.surround > MAX_SURROUND)
               levels.fields.surround = MAX_SURROUND;
         }
         if (left)
         {
            levels.fields.surround -= cnt1;
            cnt1   = 0;
            left   = false;
            if (levels.fields.surround < MIN_SURROUND)
               levels.fields.surround = MIN_SURROUND;
         }
         set_surround();
         break;
      }
      default:
         level_state = LEVEL_STATE_VOLUME;
         break;
   }

   // Reset inactivity timer
   inactivity_countdown = INACTIVITY_COUNT_INIT;
}


//===================================
// flash functions
//===================================

bool flash_validate(u8_t a, u8_t b)
{
   bool valid = false;
   u8_t c;

   c = ~a;

   if (c == b)
      valid = true;

#if SERIAL_ENABLE
      sprintf(ser_msg,"Flash validate %02x %02x %d", a, b, (int)valid);
      Serial.println(ser_msg);
#endif


   return valid;
}

void flash_restore_defaults(void)
{
   // Use default values
   levels.fields.mute     = 0;
   levels.fields.volume   = 5;
   levels.fields.sub      = 0;
   levels.fields.balance  = 0;
   levels.fields.center   = 0;
   levels.fields.surround = 0;

   flash_update();
}

void flash_update(void)
{
   u16_t addr, k;
   u8_t  val;

   if (flash_idx > 2)
   {
      addr = flash_idx;
      for (k = 0; k < sizeof(settings_t); k++)
      {
         val = ~levels.data[k];
         EEPROM.update( addr + k,
                        levels.data[k]);
         EEPROM.update( addr + sizeof(settings_t) + k,
                        val);
#if SERIAL_ENABLE
      sprintf(ser_msg,"Flash update %d %02x %02x", (addr + k), levels.data[k], val);
      Serial.println(ser_msg);
#endif
      }
   }
}

void flash_init(void)
{
   u16_t addr, k;
   bool  flash_corrupt;

   flash_idx         = EEPROM.read(0);
   flash_idx_verify  = EEPROM.read(1);

#if SERIAL_ENABLE
      sprintf(ser_msg,"Flash idx reads: %02x, %02x", flash_idx, flash_idx_verify);
      Serial.println(ser_msg);
#endif

   if ( (!flash_validate(flash_idx, flash_idx_verify)) ||
        (flash_idx < 2) )
   {
#if SERIAL_ENABLE
      Serial.print("First time initialization\n");
#endif
      // First time
      flash_idx        = 2;
      flash_idx_verify = ~flash_idx;
      EEPROM.update(0, flash_idx);
      EEPROM.update(1, flash_idx_verify);

      // Use defaults
      flash_restore_defaults();
      return;
   }

   // read levels
   flash_corrupt = false;
   addr = flash_idx;
   for (k = 0; k < sizeof(settings_t); k++)
   {
      flash_levels.data[k]        = EEPROM.read(addr + k);
      flash_levels_verify.data[k] = EEPROM.read(addr + sizeof(settings_t) + k);
      if (!flash_validate(flash_levels.data[k], flash_levels_verify.data[k]))
      {
         flash_corrupt = true;
      }
#if SERIAL_ENABLE
      sprintf(ser_msg,"Flash reads: %02x, %02x, %d", flash_levels.data[k], flash_levels_verify.data[k], (int) flash_corrupt);
      Serial.println(ser_msg);
#endif
   }

   if (flash_corrupt)
   {
      // update address to new section
      flash_idx += 2 * sizeof(settings_t);
      if (flash_idx < 2)
         flash_idx = 2;
      flash_idx_verify = ~flash_idx;
      EEPROM.update(0, flash_idx);
      EEPROM.update(1, flash_idx_verify);

      // fallback: use defaults
      flash_restore_defaults();

#if SERIAL_ENABLE
      sprintf(ser_msg,"Flash idx moved to: %d and defaults restored", flash_idx);
      Serial.println(ser_msg);
#endif
      return;
   }

#if SERIAL_ENABLE
      Serial.println("Flash read: All OK");
#endif
   // restored settings are OK, let's use them
   memcpy(&levels, &flash_levels, sizeof(settings_t));
}


//===================================
// Misc functions
//===================================

void splash (void)
{
   for(int k = 0; k < 8; k++)
   {
      lcd_clear_buffer();
      strncpy( &lcd_buffer.lin.line0[k + 0], "Here's ", 6);
      strncpy( &lcd_buffer.lin.line1[18 - k], "Johnny ", 6);
      lcd_update_display();
      delay(150);
   }
   delay(500);
}


void show_quote(void)
{
   lcd_clear_buffer();
   strncpy( &lcd_buffer.lin.line0[10], "Enneh  ", 6);
   strncpy( &lcd_buffer.lin.line1[3],  "Manneh ", 6);
   update_screen  = true;
}

void screen_update(void)
{
   lcd_update_display();
}


//===================================
// MAIN functions
//===================================

void setup()
{
   pinMode(PIN_ON,   OUTPUT);
   pinMode(PIN_MUTE, OUTPUT);
   digitalWrite(PIN_ON,   HIGH);
   digitalWrite(PIN_MUTE, HIGH);

   button_init();
   spi_init();
   i2c_init();

#if SERIAL_ENABLE
   Serial.begin (9600);
#endif

   njw1150_init();
   delay(500);
   lcd_init();
   delay(500);
   digitalWrite(PIN_ON, LOW);

   splash();
   rotary_init();
   flash_init();
   update_njw1150 = true;

   inactivity_countdown = INACTIVITY_COUNT_INIT;
}

void loop()
{
   static u16_t loop_cnt = 0;
   button_task();

   if ( right ||
        left  ||
        button_level_pressed ||
        button_mute_pressed)
   {
      handle_level();
   }

   if (update_screen)
   {
      if (!(loop_cnt & 0x03))
      {
         screen_update();
         update_screen = false;
      }
   }

   if (update_njw1150)
   {
      if (!(loop_cnt & 0x07))
      {
         njw1150_set_volume();
         njw1150_set_sub();
         njw1150_set_balance();
         njw1150_set_center();
         njw1150_set_surround();
         update_njw1150 = false;
      }
   }

   if (inactivity_countdown)
   {
      inactivity_countdown--;
      if (!inactivity_countdown)
      {
         flash_update();
         show_quote();
      }
   }
   delay(LOOP_IDLE_PERIOD);
   loop_cnt++;
}

