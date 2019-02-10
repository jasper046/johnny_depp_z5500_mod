//===================================
// definitions
//===================================

#define CS     10
#define SDA    11
#define SCL    12

#define LCD_BUFFER_SIZE         (10*8)
#define LCD_LINE_SIZE           (LCD_BUFFER_SIZE / 2)
#define LCD_LINE_VISIBLE_LENGTH (20)

typedef unsigned char u8_t;


//===================================
// global variables
//===================================

char lcd_buffer[LCD_BUFFER_SIZE];


//===================================
// SPI functions
//===================================

void spi_init(void)
{
   pinMode(CS, OUTPUT);
   digitalWrite(CS, HIGH);

   pinMode(SDA, OUTPUT);
   digitalWrite(SDA, LOW);

   pinMode(SCL, OUTPUT);
   digitalWrite(SCL, HIGH);
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
      lcd_buffer[k] = 0x20; // space
   }
}

void lcd_cls(void)
{
  spi_write(0x38);
  spi_write(0x3C);
  spi_write(0x07);

  spi_write(0x38); // '38' switch to command register??
  spi_write(0x0C); // Display on, cursor off
  spi_write(0x01); // clear display
  spi_write(0x02); // cursor to first position
  delayMicroseconds(50);
}

void lcd_init(void)
{
  lcd_cls();
  lcd_clear_buffer();
  lcd_cls();
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
   for (int m = 0; m < 10; m++)
   {
      spi_write(0x3C); // '3C' switch to data register?
      for(int k = 0; k < 8; k++)
      {
        lcd_write_char(lcd_buffer[pos + k]);
      }
      pos += 8;
   }
}




//===================================
// MAIN functions
//===================================

void setup()
{
  spi_init();
  delay(2000);
  lcd_init();
}

void loop()
{
   for(int k = 0; k < 8; k++)
   {
      lcd_clear_buffer();
      strncpy( &lcd_buffer[k], "Here's ", 6);
      strncpy( &lcd_buffer[LCD_LINE_SIZE + k + 5], "Johnny ", 6);
      lcd_update_display();
      delay(500);
   }

   lcd_clear_buffer();
   strncpy( &lcd_buffer[1], "Hallo  ", 6);
   strncpy( &lcd_buffer[LCD_LINE_SIZE + 2], "Manneh ", 6);
   lcd_update_display();
   delay(2000);
}

