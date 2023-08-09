//Display
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
int dt = 1000;

// Light Sensor
int LDRInput = A0;  //Set Analog Input A0 for LDR.
int highest_light = analogRead(LDRInput);
float shade_factor = 0.6;

// Speed Sensor
int speed_pin = 3;     // The pin the encoder is connected
float rpm;      // rpm reading
volatile byte pulses;  // number of pulses
unsigned long timeold;
// The number of pulses per revolution
// depends on your index disc!!
unsigned int pulsesperturn = 20;
float max_rpm = 9;



// Temperature Sensor Tehermistor
int ThermistorPin = 2;
int Vo;
float R1 = 10000;
float logR2, R2, T0, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
float highest_temp = 0;


//Pressure Sensor
int fsrPin = 1;       // the FSR and 10K pulldown are connected to a1
int fsrReading;       // the analog reading from the FSR resistor divider
int fsrVoltage;       // the analog reading converted to voltage
float fsrResistance;  // The voltage converted to resistance
float fsrForce;       // The resistance converted to force
float fsrPressure;    // Froce to pressure in Psi
float highest_pressure=0;

// Flow Sensor
const int flowPin = 2;        // Pin connected to the sensor's output
unsigned int pulseCount = 0;  // Counter for the number of pulses
float flowRate = 0.0;         // Flow rate in L/min
volatile unsigned long lastTime;
float calibrationFactor = 12.5;  // Calibration factor to convert pulses to flow rate (adjust this value as per your sensor)
float highest_flow = 0;

// Buzzer
const int buzzer = 9;


// Sensor which is active now;

int current_sensor = 0;
bool sensor_running = false;
void setup() {

  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  pinMode(LDRInput, INPUT);
  // HT.begin();
  delay(dt/2);

  ////// initial value of light
  for (int i = 0; i < 4; i++) {
    lcd_display_string(0, 0, "Initializing...");
    highest_light = max(highest_light, light());
    delay(100);
    lcd.clear();    
  }
 

  // Flow
  pinMode(flowPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowPin), pulseCounter, FALLING);
  lastTime = millis();

  //Speed
  pinMode(speed_pin, INPUT);
  attachInterrupt(1, counter, FALLING);
  // Initialize
  pulses = 0;
  rpm = 0;
  timeold = 0;

  //buzzer
  pinMode(buzzer, OUTPUT);
}

void loop() {
  float current_light = light();
  if (current_light < highest_light * shade_factor && !sensor_running) {
    current_sensor += 1;
    sensor_running = true;
    
  }

  if (current_sensor == 0) {
    lcd_display_string(0, 0, " Start? ");
  }

if(sensor_running){  
   if (current_sensor == 1) {
    speed();
  }
  else if (current_sensor == 2) {
    pressure();
  } 
  else if (current_sensor == 3) {
    flow();
  } 
  else if (current_sensor == 4) {
    temperature();
  }
  else if (current_sensor == 5){
    score();
  }

  led();

  digitalWrite(9, LOW);
}

  Serial.println(current_sensor);

}





float light() {
  return analogRead(LDRInput);  //Reads the Value of LDR(light)
}

void led(){
  digitalWrite(9, LOW);
}

void speed() {
  if (millis() - timeold >= 1000) {  //Uptade every one second, this will be equal to reading frecuency (Hz).

    //Don't process interrupts during calculations
    detachInterrupt(0);
    //Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt
    //happened once per revolution
    rpm = (60 * 1000 / pulsesperturn) / (millis() - timeold) * pulses;
    timeold = millis();
    pulses = 0;

    //Write it out to serial port
    lcd.clear();
    lcd_display_string(0, 0, "Swipe Speed/min");
    lcd_display_float(6, 1, rpm);

    if (rpm >= 16) {
      digitalWrite(9, HIGH);

      lcd.clear();
      lcd_display_string(0, 0, " Passed!");
      max_rpm = rpm;
      delay(dt * 4);
      sensor_running = false;
    }
    //Restart the interrupt processing
    attachInterrupt(0, counter, FALLING);
  }
}

void temperature() {
  
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T0 = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T0 - 273.15;
  highest_temp = max(Tc, highest_temp);

  if (highest_temp>28){
      digitalWrite(9, HIGH);
  }

    Serial.println(" C");   
    lcd.clear();
    lcd_display_string(0, 0, "Temperature: ");
    lcd_display_float(4, 1, Tc);
    lcd_display_string(10, 1, "C");

    if(highest_temp >= Tc + 1.5){
      sensor_running = false;
      lcd.clear();
      lcd_display_string(0, 0, "Highest Temp: ");
      lcd_display_float(7, 1, highest_temp);
      delay(4*dt);
  }

  delay(500);
}

void pressure() {
  fsrReading = analogRead(fsrPin);

  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);

  if (fsrVoltage == 0) {
    lcd.clear();
    lcd_display_string(0, 0, "No Pressure");
    fsrPressure = 0;

  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;  // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10;                // 10K resistor
    fsrResistance /= fsrVoltage;
    // Serial.print("FSR resistance in kilo-ohms = ");
    // Serial.println(fsrResistance);
    fsrForce = pow((fsrResistance / 153.18), (1 / -0.699));  // in gram
    fsrForce *= 0.00220462;                                  // in lbs
    fsrPressure = fsrForce / 0.2559519;                      // in psi

    lcd.clear();
    lcd_display_string(0, 0, "Pressure: ");
    lcd_display_float(4, 1, fsrPressure);
    lcd_display_string(10, 1, "psi");
  }
  highest_pressure = max(fsrPressure, highest_pressure);
  
  if (highest_pressure>12){
      digitalWrite(9, HIGH);
      }

  if (fsrPressure + 4 <= highest_pressure) {
    lcd.clear();
    lcd_display_string(0, 0, "Highest Pressure: ");
    lcd_display_float(4, 1, highest_pressure);
    lcd_display_string(10, 1, "psi");
    delay(dt * 3);
    sensor_running = false;
  }
  delay(dt);
}

void flow() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;

  if (elapsedTime > 1000) {  // Calculate flow rate every 1 second
    detachInterrupt(digitalPinToInterrupt(flowPin));

    // Conversion from pulses to flow rate
    flowRate = 15 * ((1000.0 / elapsedTime) * pulseCount ) * 0.0353147 / calibrationFactor;

    highest_flow = max(flowRate, highest_flow);
    lcd.clear();
    lcd_display_string(0, 0, "Flow Rate: ");
    lcd_display_float(3, 1, flowRate);
    lcd_display_string(9, 1, "L/min");

    if (highest_flow>15){
      digitalWrite(9, HIGH);
      }
  
    if (highest_flow - flowRate >= 5) {
      lcd.clear();
      lcd_display_string(0, 0, "Highest Flow: ");
      lcd_display_float(3, 1, highest_flow);
      lcd_display_string(9, 1, "L/min");
      sensor_running = false;
    }

    pulseCount = 0;          // Reset the pulse count
    lastTime = currentTime;  // Update the last time

    attachInterrupt(digitalPinToInterrupt(flowPin), pulseCounter, FALLING);
  }
}

void score(){
  int max_score =0;
  max_score += map(max_rpm, 16, 30, 0, 15);
  max_score += map(highest_pressure, 0, 25, 0, 30);
  max_score += map(highest_flow, 0, 30, 0, 30);
  max_score += map(highest_temp, 20, 38, 0, 25);
  if (max_score>60){
      digitalWrite(9, HIGH);
      }
  lcd_display_string(0, 0, "Your score:   ");
  lcd_display_float(3, 1, max_score);
  lcd_display_string(8, 1, "/100 ");

  
}


void lcd_display_string(int col, int row, String display_output) {
  lcd.setCursor(col, row);
  lcd.print(display_output);
}
void lcd_display_float(int col, int row, float display_output) {
  lcd.setCursor(col, row);
  lcd.print(display_output);
}

//Speed
void counter() {
  pulses++;
}
//Flow
void pulseCounter() {
  pulseCount++;  
}

void done_buzzer(){
    tone(buzzer, 900); // Send 1KHz sound signal...
  delay(500);       // ...for 1sec
}
