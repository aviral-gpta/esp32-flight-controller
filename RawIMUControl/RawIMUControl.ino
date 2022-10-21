#include <Wire.h>
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0, acc_x_cal, acc_y_cal, acc_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;


#define LED 9  // The pin the LED is connected to

#include <SPI.h>             /* to handle the communication interface with the modem*/
#include <nRF24L01.h>        /* to handle this particular modem driver*/
#include <RF24.h>            /* the library which helps us to control the radio modem*/
RF24 radio(7,8);             /* Creating instance 'radio'  ( CE , CSN )   CE -> D7 | CSN -> D8, SCK=52 , MISO=50 , MOSI=51 */                               
const byte Address[6] = "00001"; /* Address from which data to be received */
int throttle ;                    /* Variable to store received data */

#include <Servo.h>       //for motor
Servo motor1;
Servo motor2; 
Servo motor3;
Servo motor4;
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN1 2
#define MOTOR_PIN2 3
#define MOTOR_PIN3 4
#define MOTOR_PIN4 5
float P1, P2, P3, P4;

//PID controller
//Roll
float pid_p_gain_roll = 3.00;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.00;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 00.00;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)
float pid_roll_setpoint = 0;               //refernce angle
float pid_error_temp;  
float pid_i_mem_roll, pid_output_roll, pid_last_roll_d_error;

//Pitch
float pid_p_gain_pitch = pid_p_gain_roll;               //Gain setting for the roll P-controller
float pid_i_gain_pitch = pid_i_gain_roll;              //Gain setting for the roll I-controller
float pid_d_gain_pitch = pid_d_gain_roll;              //Gain setting for the roll D-controller
int pid_max_pitch = pid_max_roll;                    //Maximum output of the PID-controller (+/-)
float pid_pitch_setpoint = 0;               //refernce angle  
float pid_i_mem_pitch, pid_output_pitch, pid_last_pitch_d_error;

//Yaw
float pid_p_gain_yaw = 3.00;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.00;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.00;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
float pid_yaw_setpoint = 0;               //refernce angle  
float pid_i_mem_yaw, pid_output_yaw, pid_last_yaw_d_error;

//loop time measurement
float previous_time, current_time, loop_time;

void setup() {
  Serial.begin(57600);
  Wire.begin();                      // Initialize comunication
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  digitalWrite(13, HIGH);   

  IMU_calibration();

  pinMode(LED, OUTPUT); // Declare the LED as an output

  radio.begin();                   /* Activate the modem*/
  radio.openReadingPipe(1, Address); /* Sets the address of receiver from which program will receive the data*/
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();          /*Setting modem in Receiver mode*/

  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);
  
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  delay(3000);

  digitalWrite(13, LOW);                                               //All done, turn the LED off
  
  loop_timer = micros();                                               //Reset the loop timer
}

void loop() {
 
 wireless_data();
  // Serial.print("1");

 read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
  // Serial.print("2");

 euler_angles();
  // Serial.println("3");

 pid_contrl();
 
 motor_mixing();
 
 action();

 while(micros() - loop_timer < 10000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
 loop_timer = micros();                                               //Reset the loop timer
 
}

void euler_angles() {
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0001527 = (1 /100Hz) / 65.5
  angle_pitch += gyro_x * 0.0001527;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0001527;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_yaw += gyro_z * 0.0001527;
  
  //0.000002665 = 0.0001527 * (3.142(PI) / 180degr) The Arduino sin function is in radians 
  angle_pitch += angle_roll * sin(gyro_z * 0.000002665);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000002665);               //If the IMU has yawed transfer the pitch angle to the roll angel

//Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle

  if(acc_total_vector < 3600 && acc_total_vector > 3400 ){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.995 + angle_pitch_acc * 0.005;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.995 + angle_roll_acc * 0.005;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }

  Serial.print(angle_pitch);
  Serial.print("/");
  Serial.print(angle_roll);
  Serial.print("/");
  Serial.print(angle_yaw);
  Serial.print("/");
  Serial.print(acc_total_vector);
  Serial.print("/");
}

void pid_contrl() {
  //Roll calculations
  pid_error_temp = angle_roll - pid_roll_setpoint;           //white= clockwise
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = angle_pitch - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = angle_yaw - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
  pid_last_yaw_d_error = pid_error_temp;   
  
  Serial.print("\t");
  Serial.print("pid_output_ypr");
  Serial.print("\t");
  Serial.print(pid_output_yaw);    
  Serial.print("\t");
  Serial.print(pid_output_pitch);
  Serial.print("\t");
  Serial.print(pid_output_roll);
  Serial.print("\t");
}

void wireless_data() {
  if (radio.available()){
    radio.read(&throttle, sizeof(throttle));/* Read the received data and store in ' rx_data ' */
    throttle = map( throttle , 0 , 255 , 1000 , 2000 )  ;
    //Serial.print("Rvd Data : ");
   Serial.print("\t");
    Serial.print(throttle);
    Serial.print("\t");          
    }
}

void motor_mixing() {
   P1 = throttle - pid_output_roll - pid_output_pitch + pid_output_yaw;
   P2 = throttle - pid_output_roll + pid_output_pitch - pid_output_yaw;
   P3 = throttle + pid_output_roll + pid_output_pitch + pid_output_yaw;
   P4 = throttle + pid_output_roll - pid_output_pitch - pid_output_yaw;

  if(P1 < 1000)P1 = 1000;
    else if(P1 > 2000)P1 = 2000;
  if(P2 < 1000)P2 = 1000;
    else if(P2 > 2000)P2 = 2000;
  if(P3 < 1000)P3 = 1000;
    else if(P3 > 2000)P3 = 2000;
  if(P4 < 1000)P4 = 1000;
    else if(P4 > 2000)P4 = 2000;
   
   Serial.print("\t");    
    Serial.print(P1);    
    Serial.print("\t");
    Serial.print(P2);
    Serial.print("\t");
    Serial.print(P3);
    Serial.print("\t");
    Serial.print(P4);
}

void action() {
  if(throttle > 1700)digitalWrite(LED, HIGH); // Turn the LED on
  else digitalWrite(LED, LOW);

  motor1.writeMicroseconds(P1);
  motor2.writeMicroseconds(P2);
  motor3.writeMicroseconds(P3);
  motor4.writeMicroseconds(P4);
  
 Serial.print("\t");
  Serial.println("vroom");    
}


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void IMU_calibration() {
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  acc_x_cal /= 2000;  
  acc_y_cal /= 2000;  
  acc_z_cal /= 2000;  
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x_cal*acc_x_cal)+(acc_y_cal*acc_y_cal)+(acc_z_cal*acc_z_cal));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch = asin((float)acc_y_cal/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll = asin((float)acc_x_cal/acc_total_vector)* -57.296;       //Calculate the roll angle
  Serial.print("pitch_ini = ");
  Serial.println(angle_pitch);
  Serial.print("roll_ini = ");
  Serial.println(angle_roll);
  Serial.print("acc_total_vector_ini = ");
  Serial.println(acc_total_vector);  

}
