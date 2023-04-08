#define DATA0 8
#define DATA1 9
#define DATA2 2
#define DATA3 3
#define DATA4 4
#define DATA5 5
#define DATA6 6
#define DATA7 7

#define LCD_RST A4
#define LCD_CS  A3
#define LCD_RS  A2
#define LCD_WR  A1
#define LCD_RD  A0

#define COMMAND 0
#define DATA 1

void lcdWrite(byte v, bool rs) {
  #define B(X) (v & (1<<X))
  digitalWrite(LCD_RS, rs);
  //digitalWrite(LCD_CS, LOW);
  digitalWrite(DATA0, B(0));
  digitalWrite(DATA1, B(1));
  digitalWrite(DATA2, B(2));
  digitalWrite(DATA3, B(3));
  digitalWrite(DATA4, B(4));
  digitalWrite(DATA5, B(5));
  digitalWrite(DATA6, B(6));
  digitalWrite(DATA7, B(7));
  
  digitalWrite(LCD_WR, LOW);
  digitalWrite(LCD_WR, HIGH);
  
  //digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, DATA);
  #undef B
}

byte lcdRead() {
  pinMode(DATA0, INPUT);
  pinMode(DATA1, INPUT);
  pinMode(DATA2, INPUT);
  pinMode(DATA3, INPUT);
  pinMode(DATA4, INPUT);
  pinMode(DATA5, INPUT);
  pinMode(DATA6, INPUT);
  pinMode(DATA7, INPUT);

  digitalWrite(LCD_RD, LOW);
  
  delay(100);
  #define R(X) (digitalRead(DATA##X) << X)
  byte v = R(0) | R(1) | R(2) | R(3) | R(4) | R(5) | R(6) | R(7);
  digitalWrite(LCD_RD, HIGH);
  delay(100);
  pinMode(DATA0, OUTPUT);
  pinMode(DATA1, OUTPUT);
  pinMode(DATA2, OUTPUT);
  pinMode(DATA3, OUTPUT);
  pinMode(DATA4, OUTPUT);
  pinMode(DATA5, OUTPUT);
  pinMode(DATA6, OUTPUT);
  pinMode(DATA7, OUTPUT);

  return v;
  #undef R
}

uint16_t lcdRead16() {
  byte high = lcdRead();
  byte low = lcdRead();
  return (high<<8) | low;
}

void lcdWrite16(uint16_t value, bool command) {
  lcdWrite(value >> 8, command);
  lcdWrite(value & 0xFF, command);
}

#define TFTLCD_DELAY 0xFFFF

void tft_init() {
  static const uint16_t registers[] PROGMEM = {
                0x00e5,
                0x8000,
                0x0000,
                0x0001,
                0x0001,
                0x100,
                0x0002,
                0x0700,
                0x0003,
                0x1030,
                0x0004,
                0x0000,
                0x0008,
                0x0202,
                0x0009,
                0x0000,
                0x000A,
                0x0000,
                0x000C,
                0x0000,
                0x000D,
                0x0000,
                0x000F,
                0x0000,
                //-----Power On sequence-----------------------
                0x0010,
                0x0000,
                0x0011,
                0x0007,
                0x0012,
                0x0000,
                0x0013,
                0x0000,
                TFTLCD_DELAY,
                50,
                0x0010,
                0x17B0,  // SAP=1, BT=7, APE=1, AP=3
                0x0011,
                0x0007,  // DC1=0, DC0=0, VC=7
                TFTLCD_DELAY,
                10,
                0x0012,
                0x013A,  // VCMR=1, PON=3, VRH=10
                TFTLCD_DELAY,
                10,
                0x0013,
                0x1A00,  // VDV=26
                0x0029,
                0x000c,  // VCM=12
                TFTLCD_DELAY,
                10,
                //-----Gamma control-----------------------
                0x0030,
                0x0000,
                0x0031,
                0x0505,
                0x0032,
                0x0004,
                0x0035,
                0x0006,
                0x0036,
                0x0707,
                0x0037,
                0x0105,
                0x0038,
                0x0002,
                0x0039,
                0x0707,
                0x003C,
                0x0704,
                0x003D,
                0x0807,
                //-----Set RAM area-----------------------
                0x0060,
                0xA700,  // GS=1
                0x0061,
                0x0001,
                0x006A,
                0x0000,
                0x0021,
                0x0000,
                0x0020,
                0x0000,
                //-----Partial Display Control------------
                0x0080,
                0x0000,
                0x0081,
                0x0000,
                0x0082,
                0x0000,
                0x0083,
                0x0000,
                0x0084,
                0x0000,
                0x0085,
                0x0000,
                //-----Panel Control----------------------
                0x0090,
                0x0010,
                0x0092,
                0x0000,
                0x0093,
                0x0003,
                0x0095,
                0x0110,
                0x0097,
                0x0000,
                0x0098,
                0x0000,
                //-----Display on-----------------------
                0x0007,
                0x0173,
                TFTLCD_DELAY,
                50,
            };
    uint16_t s = sizeof(registers);
    uint16_t *p = (uint16_t *)registers;
    while (s > 0) {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else {
          digitalWrite(LCD_CS, LOW);
            lcdWrite16(cmd, COMMAND);
            lcdWrite16(d, DATA);
            digitalWrite(LCD_CS, HIGH);
        }
        s -= 2 * sizeof(int16_t);
    }
}




#define TFT_WRITE_GRAM 0x22
#define TFT_WIN_HSTART 0x50
#define TFT_WIN_HEND 0x51
#define TFT_WIN_VSTART 0x52
#define TFT_WIN_VEND 0x53
#define TFT_GRAM_HSET 0x20
#define TFT_GRAM_VSET 0x21

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial.println("TFT");
  pinMode(DATA0, OUTPUT);
  pinMode(DATA1, OUTPUT);
  pinMode(DATA2, OUTPUT);
  pinMode(DATA3, OUTPUT);
  pinMode(DATA4, OUTPUT);
  pinMode(DATA5, OUTPUT);
  pinMode(DATA6, OUTPUT);
  pinMode(DATA7, OUTPUT);

  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  digitalWrite(LCD_RST, HIGH);
  delay(50);
  digitalWrite(LCD_RST, LOW);
  delay(150);
  digitalWrite(LCD_RST, HIGH);
  delay(120);

  lcdWrite16(0xB0, COMMAND);
  lcdWrite16(0x0, DATA);
  
  Serial.print("Library: ");
  // tft.reset();
  Serial.println("Init TFT");
  tft_init();
  Serial.println("DONE Init TFT");
  digitalWrite(LCD_CS, LOW);
  lcdWrite16(0x61, COMMAND);
  lcdWrite16(0x2, DATA);
  lcdWrite16(0x6A, COMMAND);
  lcdWrite16(0, DATA);
  lcdWrite16(0x60, COMMAND);
  lcdWrite16(0x2700 | (1<<15), DATA);
  digitalWrite(LCD_CS, HIGH);
  // tft.begin(0x5408);
  // Serial.println(tft.readID());
  // lcdInit();
  //lcdBegin();
  /*
  lcdWrite16(0x07, COMMAND);
  lcdWrite16(0x0112, DATA);*/

  digitalWrite(LCD_CS, LOW);
  lcdWrite(0, COMMAND);
  Serial.print("Display ID: 0x");
  Serial.println(lcdRead16(), HEX);
  digitalWrite(LCD_CS, HIGH);

  // tft.setRotation(0);
  // tft.invertDisplay(true);
  // tft.fillScreen(0x55);
  

  digitalWrite(LCD_CS, LOW);
  lcdWrite16(TFT_GRAM_HSET, COMMAND);
  lcdWrite16(0, DATA);
  lcdWrite16(TFT_GRAM_VSET, COMMAND);
  lcdWrite16(0, DATA);
  
  lcdWrite16(TFT_WRITE_GRAM, COMMAND);
  Serial.println("Start filling");
  for (int y=0;y<320;y++) {
    for (int x=0;x<240;x++) {
      lcdWrite16(0xFF, DATA);
    }
  }
  Serial.println("Done filling");
  

  /*lcdWrite(CMD_READ_DISPLAY_BRIGHTNESS, COMMAND);
  lcdRead();
  Serial.print("Brightness: ");
  Serial.println(lcdRead());*/
/*
  for (int i=150;i<320;i++) {
    tft.drawPixel(i, i+10, 0xA0A0);
  }*/

  for (int i=0;i<150;i++) {
    //tft.setAddrWindow(i, i, i, i);
    digitalWrite(LCD_CS, LOW);
    // lcdWrite16(0xFDA0, COMMAND);

    /*int HS = 0x50;
    lcdWrite16(HS, COMMAND);
    lcdWrite16(10, DATA);
    lcdWrite16(HS+1, COMMAND);
    lcdWrite16(10, DATA);
    int VS = 0x52;
    lcdWrite16(VS, COMMAND);
    lcdWrite16(10, DATA);
    lcdWrite16(VS+1, COMMAND);
    lcdWrite16(10, DATA);*/

    int MW = 0x22;

    /*digitalWrite(LCD_RS, COMMAND);
    write8(MW>>8);
    digitalWrite(LCD_RS, DATA);
    digitalWrite(LCD_RS, COMMAND);
    write8(MW&0xFF);
    digitalWrite(LCD_RS, DATA);*/
    lcdWrite16(MW, COMMAND);

    /*write8(0xFDA0>>8);
    write8(0xFDA0&0xFF);*/
    lcdWrite16(0xFDA0, DATA);
    lcdWrite16(0xA0FD, DATA);
    // lcdPixelSet(i, i, 0xFDA0);
    //tft.drawPixel(i, i, 0xFDA0);
      // digitalWrite(LCD_CS, HIGH);
  }

  lcdWrite16(TFT_WIN_HSTART, COMMAND);
  lcdWrite16(10, DATA);
  lcdWrite16(TFT_WIN_HEND, COMMAND);
  lcdWrite16(100, DATA);
  lcdWrite16(TFT_WIN_VSTART, COMMAND);
  lcdWrite16(10, DATA);
  lcdWrite16(TFT_WIN_VEND, COMMAND);
  lcdWrite16(100, DATA);
  lcdWrite16(TFT_GRAM_HSET, COMMAND);
  lcdWrite16(20, DATA);
  lcdWrite16(TFT_GRAM_VSET, COMMAND);
  lcdWrite16(20, DATA);

  lcdWrite16(TFT_WRITE_GRAM, COMMAND);
  for (int y=10;y<100;y++) {
    for (int x=10;x<100;x++) {
      lcdWrite16(0xFDA0, DATA);
    }
    delay(1);
  }

  Serial.println("Done drawing");
}

void loop() {
}
