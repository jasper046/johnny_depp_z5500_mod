//===================================
// definitions
//===================================

#define SPI_CS     10
#define SPI_SDA    11
#define SPI_SCL    12

#define I2C_SDA     5
#define I2C_SCL     4
#define I2C_DELAY   5
// Delay of 5 usec ~= 100kHz I2C
#define I2C_OK      0
#define I2C_ERROR   1

//BOARD	                              DIGITAL PINS USABLE FOR INTERRUPTS
//Uno, Nano, Mini, other 328-based     2, 3

// rotary encoder
#define ENCPINA 3
#define ENCPINB 2

// NJW1150 control
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

// LCS pannel
#define LCD_BUFFER_SIZE         (10*8)
#define LCD_LINE_SIZE           (LCD_BUFFER_SIZE / 2)
#define LCD_LINE_VISIBLE_LENGTH (20)

#define MAX_VOLUME              (32)

#define SERIAL_ENABLE           (0)


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

char  msg[LCD_LINE_SIZE];

//===================================
// global variables
//===================================

LcdBuf_t lcd_buffer;

volatile byte seqA = 0;
volatile byte seqB = 0;
volatile byte cnt1 = 0;
volatile byte cnt2 = 0;
volatile boolean right  = false;
volatile boolean left   = false;
volatile boolean button = false;

s16_t    volume = 0;
bool     update = false;


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
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_CENTER, 0x06);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_SUB,    0x0C);
   i2c_write_register(NJW1150_ADDR, NJW1150_REG_MUTE,   0x00);
}


void njw1150_set_volume(u8_t vol)
{
   u8_t val = 0;

   if (vol == 0)
      val = 0x50; // mute
   if (vol >= 32)
      val = 0;    // 0dB attenuation
   else
      val = 79 - (vol*5/2);

   i2c_write_register(NJW1150_ADDR, NJW1150_REG_VOLUME, val);
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
      delay(200);
   }
   delay(500);
}


void set_volume(void)
{
   u16_t vol_dif2;
   u16_t k;

   vol_dif2 = volume>>1;

   // Update display
#if SERIAL_ENABLE
   Serial.println (volume);
#endif
   lcd_clear_buffer();
   strncpy( &lcd_buffer.lin.line0[4], "Volume - ", 8);
   sprintf( msg, "%02d", volume);
   strncpy( &lcd_buffer.lin.line0[13], msg, 2);
   for(k = 0; k < vol_dif2; k++)
   {
      lcd_buffer.lin.line1[k+2] = '=';
#if SERIAL_ENABLE
      Serial.print ('X');
#endif
   }
   if (volume & 1)
   {
      lcd_buffer.lin.line1[k+2] = ':';
#if SERIAL_ENABLE
      Serial.print ('x');
#endif
   }
#if SERIAL_ENABLE
   Serial.print ("\n");
#endif

   update = true;
}


void screen_update(void)
{
   if (update)
   {
      lcd_update_display();
   }
   update = false;
}


//===================================
// MAIN functions
//===================================

void setup()
{
   spi_init();
   i2c_init();

#if SERIAL_ENABLE
   Serial.begin (9600);
#endif

   delay(500);
   lcd_init();
   njw1150_init();
   delay(500);

   splash();
   rotary_init();
}

void loop()
{
   static u16_t loop_cnt = 0;

   if (right || left)
   {
      if (right)
      {
         volume += cnt2;
         cnt2   = 0;
         right  = false;
         if (volume > MAX_VOLUME)
            volume = MAX_VOLUME;

         set_volume();
      }

      if (left)
      {
         volume -= cnt1;
         cnt1   = 0;
         left   = false;
         if (volume < 0)
            volume = 0;

         set_volume();
      }
   }

   loop_cnt++;
   if (update)
   {
      if (!(loop_cnt & 0x03))
         screen_update();

      if (!(loop_cnt & 0x07))
         njw1150_set_volume(volume);
   }
   delay(25);
}

