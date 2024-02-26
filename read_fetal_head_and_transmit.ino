// Arduino code for measurement of the force applied on the scalp. 
// fetal_head_pressure_gb.ino 

//Initialisation of variables 

#define S0 10 
#define S1 11 
#define S2 12 
#define S3 13 

#define B_DATA_PIN A0
#define R_DATA_PIN A1
#define L_DATA_PIN A2
#define F_DATA_PIN A3

//Address’ of the multiplexer pins 
const bool Numbers [16] [4] = {{0, 0, 0, 0}, //0 
{1, 0, 0, 0}, //1 
{0, 1, 0, 0}, //2 
{1, 1, 0, 0}, //3 
{0, 0, 1, 0}, //4 
{1, 0, 1, 0}, //5 
{0, 1, 1, 0}, //6 
{1, 1, 1, 0}, //7 
{0, 0, 0, 1}, //8 
{1, 0, 0, 1}, //9 
{0, 1, 0, 1}, //10 
{1, 1, 0, 1}, //11 
{0, 0, 1, 1}, //12 
{1, 0, 1, 1}, //13 
{0, 1, 1, 1}, //14 
{1, 1, 1, 1}}; //15 

void setup() 
{ 
  //Defintion of the four outputs of the Arduino 
  pinMode(S0, OUTPUT); 
  pinMode(S1, OUTPUT); 
  pinMode(S2, OUTPUT); 
  pinMode(S3, OUTPUT); 

  //Initialisation of serial communication 
  Serial.begin(115200); 
} 

//Function that open the chosen channel of the multiplexor 
void selectMuxPin(int channel)
{ 
  const int selectPins[4] = {10, 11, 12, 13}; 
  if (channel > 16) return; // Exit if pin is out of scope 
  
  for (int i = 0; i < 4; i++) 
  { 
    if (Numbers[channel][i] == 1)
    {
      digitalWrite(selectPins[i], HIGH); 
    }
    else
    { 
      digitalWrite(selectPins[i], LOW); 
    }  
  } 
} 

void blocking_write(byte input)
{
  while (Serial.write(input) == 0)
  {}
}

void send_over_serial(int input)
{
  blocking_write(highByte(input));
  blocking_write(lowByte(input));
}

void loop() 
{ 
  const int CHANNELS_PER_MUX = 16;

  send_over_serial(0xDEAD);
  send_over_serial(0xBEEF);
  send_over_serial(0xC001);
  send_over_serial(0xCAFE);

  for (int j = 0; j < CHANNELS_PER_MUX; j++) 
  { 
    selectMuxPin(j); //Open j channel of each multiplexer at the same time 
    send_over_serial(analogRead(B_DATA_PIN)); 
  } 
  for (int j = 0; j < CHANNELS_PER_MUX; j++) 
  { 
    selectMuxPin(j); //Open j channel of each multiplexer at the same time 
    send_over_serial(analogRead(R_DATA_PIN)); 
  } 
  for (int j = 0; j < CHANNELS_PER_MUX; j++) 
  { 
    selectMuxPin(j); //Open j channel of each multiplexer at the same time 
    send_over_serial(analogRead(L_DATA_PIN)); 
  } 
  for (int j = 0; j < CHANNELS_PER_MUX; j++) 
  { 
    selectMuxPin(j); //Open j channel of each multiplexer at the same time 
    send_over_serial(analogRead(F_DATA_PIN)); 
  } 
}