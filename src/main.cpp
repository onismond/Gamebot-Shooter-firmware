#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <BleGamepad.h>


//ABXY BUTTONS
#define X_BUTTON 33         // A
#define CIRCLE_BUTTON 0     // B
#define TRIANGLE_BUTTON 0   // Y
#define SQUARE_BUTTON 0     // X

//TRIGGERS
#define R1_BUTTON 0
#define R2_BUTTON 18
#define L1_BUTTON 0
#define L2_BUTTON 0

//MENU BUTTONS
#define START_BUTTON 0
#define SELECT_BUTTON 0
#define PS_BUTTON 0

//JOYSTICKS BUTTONS
#define R3_BUTTON 0
#define L3_BUTTON 0

//JOYSTICKS
#define LEFT_VRX_JOYSTICK 35
#define LEFT_VRY_JOYSTICK 34
#define RIGHT_VRX_JOYSTICK 0
#define RIGHT_VRY_JOYSTICK 0

#define NUM_BUTTONS 13
//The order of these three arrays matters a lot, be carefully when changing them
int buttonsPins[NUM_BUTTONS] = {X_BUTTON, CIRCLE_BUTTON, TRIANGLE_BUTTON, SQUARE_BUTTON,
                          R1_BUTTON, R2_BUTTON, L1_BUTTON, L2_BUTTON,
                          START_BUTTON, SELECT_BUTTON, PS_BUTTON,
                          R3_BUTTON, L3_BUTTON};
//There is not buttons for Y and Z
int androidGamepadButtons[NUM_BUTTONS] = {1, 2, 3, 4, 8, 10, 7, 9, 12, 11, 13, 15, 14};
int PS1GamepadButtons[NUM_BUTTONS] = {2, 3, 4, 1, 6, 8, 5, 7, 10, 9, 13, 12, 11};
int PCGamepadButtons[NUM_BUTTONS] = {1, 2, 4, 3, 6, 8, 5, 7, 10, 9, 0, 12, 11};

// uint16_t leftVrxJoystickLecture = 0;
// uint16_t leftVryJoystickLecture = 0;
// uint16_t rightVrxJoystickLecture = 0;
// uint16_t rightVryJoystickLecture = 0;
// uint16_t leftVrxJoystickValue = 0;
// uint16_t leftVryJoystickValue = 0;
// uint16_t rightVrxJoystickValue = 0;
// uint16_t rightVryJoystickValue = 0;

typedef enum{ANDROID, PS1, PC} GamepadModes;
GamepadModes gamepadMode = PC;

// MPU control/status vars
bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// FreeRTOS
static const BaseType_t app_cpu = 1;
#define JOYSTICK_ARRAY_SIZE 2
static const uint8_t queue_len = 20;
static QueueHandle_t keyQueue;
static QueueHandle_t mpuQueue;
static QueueHandle_t joystickQueue;

MPU6050 mpu;
BleGamepad bleGamepad("VR Shooter", "GameBot");
BleGamepadConfiguration bleGamepadConfig;  

void joysticksHandlerForMobile(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry);
void joysticksHandlerForPC(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry);

void sendGamepadKeyPress(void *parameters) {
    int16_t rightJoystickData[JOYSTICK_ARRAY_SIZE];
    int16_t leftJoystickData[JOYSTICK_ARRAY_SIZE];
    uint16_t keyData;
    while (true) {
      if (bleGamepad.isConnected()) {
        if (xQueueReceive(mpuQueue, &rightJoystickData, pdMS_TO_TICKS(0)) == pdTRUE) {
          bleGamepad.pressStart();
          bleGamepad.setRightThumb(rightJoystickData[0], rightJoystickData[1]);
          // bleGamepad.setRX(rightJoystickData[0]);
          // bleGamepad.setRY(rightJoystickData[1]);
          Serial.printf("MPU joystick value: %d, %d\n", rightJoystickData[0], rightJoystickData[1]);
        }

        if (xQueueReceive(joystickQueue, &leftJoystickData, pdMS_TO_TICKS(0)) == pdTRUE) {
          bleGamepad.pressStart();
          bleGamepad.setX(leftJoystickData[0]);
          bleGamepad.setY(leftJoystickData[1]);
          Serial.printf("Left joystick value: %d, %d \n", leftJoystickData[0], leftJoystickData[1]);
        }

        if (xQueueReceive(keyQueue, &keyData, pdMS_TO_TICKS(0)) == pdTRUE) {
          bleGamepad.pressStart();
          if (keyData == 41) {
            bleGamepad.pressStart();
            // bleGamepad.setTriggers(0, 255);
            bleGamepad.press(11);
            Serial.println("RT pressed");
          } else if (keyData == 40) {
            // bleGamepad.pressStart();
            bleGamepad.release(11);
            Serial.println("RT released");
          }
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        bleGamepad.sendReport();
      }
      
      vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void readMPUData(void *parameters) {
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  if (!dmpReady) return;
  float x_old = 0;
  float y_old = 0;
  int16_t joystick_sensitivity = 100;
  int16_t mid_value = 16368;

  bool is_negating = false;
  unsigned long negation_start_time = 0;
  int negation_duration = 10;
  int x_accumulated_tilt = 0;
  int stabilization_time = 50;
  int neutral_threshold = 10;
  // int16_t joystick_sensitivity = 1;
  unsigned long tilt_start_time = 0;
  unsigned long last_time = millis();

  while (true) {
    // Check for DMP data
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      unsigned long current_time = millis();
      unsigned long dt = current_time - last_time;
      last_time = current_time;

      // Get Quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // Convert radians to degrees
      float yaw   = ypr[0] * 180/M_PI;
      // float pitch = ypr[1] * 180/M_PI;
      float roll  = ypr[2] * 180/M_PI;
      // Serial.printf("YPR [deg]: %.2f %.2f %.2f\n", yaw, pitch , roll);
      float x = (yaw - x_old) * joystick_sensitivity;
      float y = (roll - y_old) * joystick_sensitivity;
      // Serial.printf("Joystick: %.2f \n", x);
      // Serial.printf("x_old: %.2f \n", x_old);
      x_old = yaw;
      y_old = roll;
      if (x > 500) {
        x = 500;
      } else if (x < -500) {
        x = -500;
      }
      if (y > 500) {
        y = 500;
      } else if (y < -500) {
        y = -500;
      }
      int16_t x_value = (int16_t) map(x, -500, 500, -15368, 15368);
      int16_t y_value = (int16_t) map(y, 500, -500, -15368, 15368);
      int16_t data[JOYSTICK_ARRAY_SIZE] = {x_value + mid_value, y_value + mid_value};
      // int16_t data[JOYSTICK_ARRAY_SIZE] = {x_value, y_value};


      // if (is_negating) {
      //   if ((current_time - negation_start_time) < negation_duration) {
      //     // int16_t data[JOYSTICK_ARRAY_SIZE] = {x_accumulated_tilt * joystick_sensitivity, y_value};
      //     // xQueueSend(mpuQueue, &data, pdMS_TO_TICKS(0));
      //   } else {
      //     is_negating = false;
      //     x_accumulated_tilt = 0;
      //     tilt_start_time = 0;
      //   }
      // } else {
      //   if (abs(x_value) > neutral_threshold) {
      //     if (tilt_start_time == 0) {
      //       tilt_start_time = current_time;
      //     }
      //     int16_t data[JOYSTICK_ARRAY_SIZE] = {-x_value * joystick_sensitivity, y_value};
      //     xQueueSend(mpuQueue, &data, pdMS_TO_TICKS(0));
      //     x_accumulated_tilt = x_value;
      //   } else {
      //     if ((tilt_start_time != 0) && ((current_time - tilt_start_time) >= stabilization_time)) {
      //       is_negating = true;
      //       negation_start_time = current_time;
      //       // int16_t data[JOYSTICK_ARRAY_SIZE] = {x_accumulated_tilt * joystick_sensitivity, y_value};
      //       // xQueueSend(mpuQueue, &data, pdMS_TO_TICKS(0));
      //     } else {
      //       int16_t data[JOYSTICK_ARRAY_SIZE] = {0, y_value};
      //       xQueueSend(mpuQueue, &data, pdMS_TO_TICKS(0));
      //     }
      //   }
      // }
      // xQueueSend(mpuQueue, &data, pdMS_TO_TICKS(0));
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
  
}

void readGamepadKeys(void *parameters) {
  pinMode(R2_BUTTON, INPUT);
  int old_fire_state = 1;
  int fire_state = 1;
  while (true) {
    if(bleGamepad.isConnected()){
      // uint16_t leftVrxJoystickLecture = analogRead(LEFT_VRX_JOYSTICK);
      // uint16_t leftVryJoystickLecture = analogRead(LEFT_VRY_JOYSTICK);
      // //Compute joysticks value
      // uint16_t leftVrxJoystickValue = map(leftVrxJoystickLecture, 4095, 0, 0, 32737);
      // uint16_t leftVryJoystickValue = map(leftVryJoystickLecture, 0, 4095, 0, 32737);

      // int16_t data[JOYSTICK_ARRAY_SIZE] = {leftVrxJoystickValue, leftVryJoystickValue};
      // xQueueSend(joystickQueue, &data, pdMS_TO_TICKS(0));

      fire_state = digitalRead(R2_BUTTON);
      // Serial.printf("Fire state: %d, Old: %d \n", fire_state, old_fire_state);
      if (fire_state != old_fire_state) {
        uint16_t fireButtonData;
        if (fire_state == 0) {
          fireButtonData = 41;
        } else {
          fireButtonData = 40;
        }
        // Serial.println("Fire button pressed");
        xQueueSend(keyQueue, &fireButtonData, pdMS_TO_TICKS(0));
      }
      old_fire_state = fire_state;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(9600);
  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.println();
  Serial.println("---GameBot---");

  /**** Gamepad Setup ****/
  for(int i=0; i<NUM_BUTTONS; i++){
    pinMode(buttonsPins[i], INPUT_PULLUP);
  }
  // bleGamepad.begin();
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  // bleGamepadConfig.setVid(0x045E); // Microsoft VID
  // bleGamepadConfig.setPid(0x028E); // Xbox 360 Controller PID
  bleGamepadConfig.setButtonCount(10); // Xbox 360: A, B, X, Y, LB, RB, Back, Start, L-Stick, R-Stick
  // bleGamepadConfig.setAxisCount(6); // 4 joystick axes (X, Y, RX, RY) + 2 triggers (LT, RT)
  bleGamepadConfig.setIncludeXAxis(true);
  bleGamepadConfig.setIncludeYAxis(true);
  bleGamepadConfig.setIncludeRxAxis(true);
  bleGamepadConfig.setIncludeRyAxis(true);
  bleGamepadConfig.setIncludeSlider1(true); // Left Trigger (LT)
  bleGamepadConfig.setIncludeSlider2(true); // Right Trigger (RT)
  bleGamepadConfig.setHatSwitchCount(1); // Xbox D-pad
  bleGamepad.begin(&bleGamepadConfig);

  /**** MPU Setup ****/
  Wire.begin(21, 22);  // SDA, SCL
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(29);
  mpu.setYAccelOffset(2299);
  mpu.setZAccelOffset(1649);
  mpu.setXGyroOffset(-11);
  mpu.setYGyroOffset(-8);
  mpu.setZGyroOffset(-55);

  if (devStatus == 0) {
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("DMP ready!");
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  mpuQueue = xQueueCreate(queue_len, sizeof(int16_t[JOYSTICK_ARRAY_SIZE]));
  joystickQueue = xQueueCreate(queue_len, sizeof(int16_t[JOYSTICK_ARRAY_SIZE]));
  keyQueue = xQueueCreate(queue_len, sizeof(uint8_t));
  // init_queues();
  xTaskCreate(
    sendGamepadKeyPress,
    "Send Gamepad Key Press",
    4096,
    NULL,
    1,
    NULL
  );
  xTaskCreate(
    readMPUData,
    "Read Data from MPU",
    4096,
    NULL,
    1,
    NULL
  );
  xTaskCreate(
    readGamepadKeys,
    "Read Gamepad Keys",
    4096,
    NULL,
    1,
    NULL
  );
  vTaskDelete(NULL);
}

void loop() {}

void joysticksHandlerForMobile(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry){
  bleGamepad.setLeftThumb(leftVrx, leftVry);
  bleGamepad.setRightThumb(rightVrx, rightVry);
}

void joysticksHandlerForPC(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry){
  bleGamepad.setX(leftVrx);
  bleGamepad.setY(leftVry);
  bleGamepad.setZ(rightVrx);
  bleGamepad.setRX(rightVry);
}
