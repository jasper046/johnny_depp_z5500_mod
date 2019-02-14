//===================================
// definitions
//===================================

#define CS     10
#define SDA    11
#define SCL    12

//BOARD	                              DIGITAL PINS USABLE FOR INTERRUPTS
//Uno, Nano, Mini, other 328-based     2, 3
#define ENCPINA 3
#define ENCPINB 2


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
   pinMode(SCL, OUTPUT);
   digitalWrite(SCL, LOW);

   pinMode(CS, OUTPUT);
   digitalWrite(CS, LOW);

   pinMode(SDA, OUTPUT);
   digitalWrite(SDA, LOW);
}

void spi_write(u8_t val)
{
   u8_t k;
   bool b;

   digitalWrite(CS, LOW);
   delayMicroseconds(2);
   k = 8;
   while(k--)
   {
      b = (val & 0x80) ? HIGH : LOW;
      val <<= 1;

      digitalWrite(SDA, b);
      digitalWrite(SCL, LOW);
      delayMicroseconds(2);
      digitalWrite(SCL, HIGH);
   }
   digitalWrite(CS,  HIGH);
   delayMicroseconds(60);
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

//  spi_write(0x38); // '38' switch to command register??
//  spi_write(0x40);
//  spi_write(0x20);

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

   // Record the A and B signals in seperate sequences
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
      lcd_buffer.lin.line1[k+2] = '-';
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

#if SERIAL_ENABLE
  Serial.begin (9600);
#endif

  delay(500);
  lcd_init();
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
   loop_cnt &= 0x03;
   if (!loop_cnt)
      screen_update();

   delay(25);
}

