#include <avr/eeprom.h>
#include <SoftwareSerial.h>
#include <SimpleTimer.h>

const int LISTEN = 1;
const int PROGRAM = 2;
const int WIPE = 3;
int MODE = 1;

int ledPin = 13;
int modePin = A0;
int beepPin = A4;
int lockPin = A3;
const int buttonPin = A5;

SimpleTimer timer;

int rxPin = 2;
int txPin = 3;
SoftwareSerial RFID = SoftwareSerial(rxPin, txPin);

struct configuration
{
  char tags[10][12];
  int tagCount;
};

configuration config;

char tag[12];

boolean readerEnabled = true;

void readEEPROM()
{
  eeprom_read_block((void*)&config, (void*)0, sizeof(config));
}

void writeEEPROM()
{
  eeprom_write_block((const void*)&config, (void*)0, sizeof(config));
}

void setup()
{
  readEEPROM();

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(modePin, HIGH);

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  RFID.begin(9600);

  Serial.begin(9600);
  Serial.println("init");


  pinMode(buttonPin, INPUT);
}

void disableReader()
{
  readerEnabled = false;
  digitalWrite(ledPin, HIGH);
  timer.setTimeout(2000, enableReader);
}

void enableReader()
{
  digitalWrite(ledPin, LOW);
  readerEnabled = true;
}

void openLock()
{
  digitalWrite(lockPin, HIGH);
  timer.setTimeout(2500, stopLock);
}

void stopLock()
{
  digitalWrite(lockPin, LOW);
}

void beep(int time)
{
  digitalWrite(beepPin, HIGH);
  timer.setTimeout(time, stopBeep);
}

void stopBeep()
{
  digitalWrite(beepPin, LOW);
}

boolean isPressed(int button)
{
  if (digitalRead(buttonPin))
  {
    delay(100);
    if (digitalRead(buttonPin))
    {
      while(digitalRead(buttonPin)) {   
      }
      return true;
    }
  }
  return false;
}

boolean readTag()
{
  int value = 0;
  int bytesread = 0;
  char data[14];

  while(RFID.available() > 0)
  {
    data[bytesread] = RFID.read();
    bytesread++;

    if (bytesread == 14) // if the whole data was read
    {
      bytesread = 0;
      if(data[0] == 2 && data[13] == 3) // check headers
      {
        for(int x = 1; x < 11; x++)
        {
          tag[x-1] = data[x];
        }
        if (readerEnabled)
        {
          disableReader();

          Serial.print(tag);
          Serial.println(" red");
          return true;
        }
      }
      else
      {
        Serial.println("problem reading tag");
        return false;
      }
    }
  }
  return false;
}

void loop()
{
  timer.run();

  if (isPressed(buttonPin))
  {
    MODE++;
    digitalWrite(modePin, LOW); // turn off previous mode led
    modePin++;
    if (MODE > 3) { 
      MODE = 1; 
      modePin = A0; 
    }
    digitalWrite(modePin, HIGH);
  }

  if (readTag())
  {
    if (MODE == LISTEN)
    {
      boolean allowAccess = false;
      for (int i=0; i < 10; i++)
      {
        allowAccess = true;
        for (int j = 0; j < 12; j++)
        {
          if (config.tags[i][j] != tag[j]) { 
            allowAccess = false; 
          }
        }
        if (allowAccess) { 
          break; 
        }
      }

      if (allowAccess)
      {
        Serial.print(tag);
        Serial.println(" accepted");
        openLock(); 
      }
      else
      {
        Serial.print(tag);
        Serial.println(" denied");
        beep(1000); 
      }
    }

    else if (MODE == PROGRAM)
    {
      for (int i=0; i < 12 ; i++)
      {
        config.tags[config.tagCount][i] = tag[i];
      }
      Serial.print(tag);
      Serial.print(" stored in pos ");
      Serial.println(config.tagCount);

      config.tagCount++;
      if (config.tagCount == 10) { 
        config.tagCount = 0; 
      }
      writeEEPROM();
    }

    else
    {
      for (int i=0; i < 10; i++)
      {
        for (int j = 0; j < 12; j++)
        {
          config.tags[i][j] = '\0';
        }
      }
      config.tagCount = 0;
      writeEEPROM();
      Serial.println("Wiped the hell out of it");
    }
  }
}

void printStoredTags()
{
  for (int i=0; i < 10; i++)
  {
    Serial.print(i);
    Serial.print(" - ");
    for (int j = 0; j < 12; j++)
    {
      Serial.print(config.tags[i][j]);
    }
    Serial.println();
  }
}




