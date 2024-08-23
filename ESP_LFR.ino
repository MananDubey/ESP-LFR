#include <QTRSensors.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing Arduino reset pin)

// Initialize the OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// QTR-8RC Sensor pins
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 13

// OLED Button Pins
#define BU 39
#define BD 34
#define BL 35
#define BR 36

// Motor driver pins
#define AIN1 33
#define AIN2 25
#define PWMA 32
#define STBY 26
#define BIN1 27
#define BIN2 14
#define PWMB 12

uint16_t sensorValues[NUM_SENSORS];

// PID constants
float Kp = 3.2;
float Ki = 0;
float Kd = 7.96;

// PID variables
float setPoint = 3500;  // Center of the line (sum of max sensor values / 2)
float lastError = 0;
float integral = 0;
float derivative = 0;
float error = 0;
float pidValue = 0;
int mode = 0;

// Motor speed
int baseSpeed = 200;  // Base PWM value for motors (0-255)
int maxSpeed = 255;
int leftSpeed = 0;
int rightSpeed = 0;

// sync
int currentTurn = 0;

// Mutex for shared PID variables
SemaphoreHandle_t xMutex;

// Function prototypes
void core0(void *parameter);
void core1(void *parameter);

QTRSensors qtr; // Alias

void setup() {
  bool setCpuFreqencyMhz(240);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);

  
  // Buttons
  pinMode(BU, INPUT);
  pinMode(BD, INPUT);
  pinMode(BL, INPUT);
  pinMode(BR, INPUT);
  // Set sensor type to RC mode
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {23, 19, 18, 5, 17, 16, 4, 2}, NUM_SENSORS);

  /*if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is the I2C address
    int current = 0;
    int exec = 0;
    int pressedButton = -1;
    const int buttonPins[] = {39, 34, 35, 36};
    int tru = 1;
    int pressed = 0;
    while (tru){ //oled loop
   
      // Clear the buffer
      display.clearDisplay();

      // Set text size, color, and cursor position
      display.setTextSize(2);      // Text size multiplier (1 is smallest)
      display.setTextColor(SSD1306_WHITE); // Text color
      display.setCursor(0, 10);    // Set cursor position (x, y)
      switch (current){
        case 0:
          display.println(pressedButton);
          if (exec) tru = 0;
          break;
        case 1:
          display.println("sensor data");
          if (exec) tru = 0;
          break;
        case 2:
          display.println("Calibrate");
          if (exec) tru = 0;
          break;
        case 3:
          display.println("stuff");
          if (exec) tru = 0; 
          break;  
        default:
          current = 0;
      }
      display.display();
      pressed = 0;
      while(!pressed)
       for (int i = 0; i < 4; i++) {
        Serial.print(buttonPins[i]);
        Serial.print(" ");
        Serial.println(digitalRead(buttonPins[i]));
        delay(300);
        if (digitalRead(buttonPins[i]) == LOW) {  // Button is pressed
         pressedButton = i+1;
         pressed = 1;
         break;
        }
      }
      
      if (pressedButton == BU){
        current++;
      } else if (pressedButton == BD){
        current--;
      } else if (pressedButton == BR){
        exec = 1;
      } else if (pressedButton == BL){
        exec = 0;
      }
    }   
  }*/

  // claibration
  setMotorSpeeds(20, 20);
  delay(250);
  setMotorSpeeds(13, 13);
  delay(250);
  setMotorSpeeds(0, 0);
  delay(500);
  // Calibrate sensors
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(15);
  }

  // Create mutex
  xMutex = xSemaphoreCreateMutex();

  // Create tasks pinned to specific cores
  xTaskCreatePinnedToCore(
    core0,  // Task function
    "Motor Control and Logic",    // Task name
    10000,                        // Stack size
    NULL,                         // Task parameters
    1,                            // Priority
    NULL,                         // Task handle
    0                             // Core 0
  );

  xTaskCreatePinnedToCore(
    core1,  // Task function
    "Sensor and PID",  // Task name
    10000,             // Stack size
    NULL,              // Task parameters
    1,                 // Priority
    NULL,              // Task handle
    1                  // Core 1
  );
}

void loop() {
  // leave empty
}

void core0(void *parameter) {
  int lMode, tj_chk;
  while (1) {
    // Protect shared resources with a mutex
    line:
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      pidValue = (Kp * error) + (Ki * integral) + (Kd * derivative);
      lMode = mode;
            
      xSemaphoreGive(xMutex);
      if (lMode == 1){
        // T-Junction
        // Go fwd for a couple of secs
        setMotorSpeeds(255, 255);
        vTaskDelay(250);
        tj_cDk = qtr.readLineBlack(sensorValues);
        for (int i = 0; i < 8; i++){
          if (sensorValues[i] > 50){
            goto line;
          }
        }
        // right turn till not on line :
        while(1){
        setMotorSpeeds(255, -255);
          for (int i = 0; i < 8; i++){
            if (sensorValues[i] > 50){
              goto line;
            }
          }
        }
      }

      leftSpeed = constrain(baseSpeed + pidValue, -20, maxSpeed);
      rightSpeed = constrain(baseSpeed - pidValue, -20, maxSpeed);
      setMotorSpeeds(leftSpeed, rightSpeed);

    }
  taskYIELD();
  vTaskDelay(1); 
  }
}

void core1(void *parameter) {
  float lErr, lInt, lDer;
  float lPID;
  while (1) {
    lErr = qtr.readLineBlack(sensorValues) - setPoint;
    lInt += lErr;
    lDer = lErr - lastError;
    lastError = lErr;
    mode = 0;
    for (int i=0; i<8; i++){
      if (sensorValues[i]< 900){
        goto cont;
      } 
    }
    mode = 1;
    cont:
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      error = lErr;
      integral = lInt;
      derivative = lDer;
      xSemaphoreGive(xMutex);
    }
    taskYIELD();
    vTaskDelay(1); 
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor control
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -leftSpeed);
  }

  // Right motor control
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -rightSpeed);
  }
}
