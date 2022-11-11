


/*
 * Balance beam regulation
 *
 *      by:
 * Blaafjell, Filip
 * Simengård, Martin
 * Woie Refsdal, Elias

 * 
 */


//  LIBRARIES
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <MedianFilterLib2.h>



enum READ_INFO{
  NAME,
  VALUE
} serial_read_info = NAME;
String command, serial_current_name;
float serial_read;


//  SERVO
#define SERVO_PIN 9
#define SERVO_MIN 13
#define SERVO_ZERO 55
#define SERVO_MAX 95
Servo servo;
int servo_pos;
int holdPos;
bool useHoldPos;


//  PID
double setpoint_ = 20, process_value, PID_out, PID_out_mapped, PID_out_constrained;
double error, last_error;
double P_term, I_term, D_term;
uint32_t current_time, last_time;
const double I_THRESHOLD_MAX = 1, I_THRESHOLD_MIN = 0;
double Kp = 6;
double Ki = 0.15;
double Kd = 5000;
#define SAMPLE_TIME 13

//timer
unsigned long nextTimeout;
bool timerStarted;
bool lastTimerStarted;



//  SENSOR
#define SENSOR_PIN A0
#define SENSOR_MEDIAN_FILTER 3      // Median filter window size. MUST BE ODD
double distance;


//Filter
double KF_mea = 1.2, KF_est = 2, KF_var = 0.3;
SimpleKalmanFilter filter = SimpleKalmanFilter(KF_mea, KF_est, KF_var);
MedianFilter2<double> medianFilter(5);

//timer
void startTimer (unsigned long timeout) {
  nextTimeout = millis() + timeout;
  }

bool hasTimerExpired () {
  bool expired = false;
  if (millis() >= nextTimeout) {
    expired = true;
  }
  return expired;
}

void setup() {
  Serial.begin(9600);

  pinMode(SENSOR_PIN, INPUT);
  
  // SERVO
  servo.attach(SERVO_PIN);
  servo.write((SERVO_MAX - SERVO_MIN)/2 + SERVO_MIN);

  last_time = millis();


/*
  for(int i = 50; i <=60; i+=1){
    servo.write(i);
    Serial.print("Grader: ");
    Serial.println(i);
    delay(3000);
 
  }

  */
}




void loop() {
  current_time = millis();

  /*
  if(Serial.available()){
        command = Serial.readStringUntil('\n');
        Serial.println("Command was:  " + command);
        if(serial_read_info == NAME) {
            serial_current_name = command;
            serial_read_info = VALUE;
        } else
        {
            if(serial_current_name == "p") {Kp = command.toFloat(); Serial.println("New Kp is: " + command);}
            else if(serial_current_name == "i") {Ki = command.toFloat(); Serial.println("New Ki is: " + command);}
            else if(serial_current_name == "d") {Kd = command.toFloat(); Serial.println("New Kd is: " + command);}
            else if(serial_current_name == "s") {setpoint_ = command.toFloat(); Serial.println("New SP is: " + command);}
            else {Serial.println("Upps, not valid");}
            serial_read_info = NAME;
        }     
    }
  */

    
  


  // PID
  if (current_time - last_time >= SAMPLE_TIME)
  {
    // SENSOR
  distance = getDistance(1);
  double distance_filtered = medianFilter.AddValue(distance);     // Median filter
  distance_filtered = filter.updateEstimate(distance_filtered);   // Kalman-filter  
  //double distance_filtered = medianFilter.AddValue(distance);     // Median filter
  process_value = distance_filtered;       // USE THIS FOR FILTERED DISTANCE
  //process_value = distance;                   // USE THIS FOR UN-FILTERED DISTANCE
/*
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(distance_filtered);
  */

    
    error = setpoint_ - process_value;

    float delta_error = error - last_error;
    uint32_t delta_time = current_time - last_time;

    P_term = Kp * error;
    D_term = Kd * (delta_error / delta_time);

    // Bruker I-delen bare når ballen er nærme setpoint_
    if( -I_THRESHOLD_MAX < error && error < I_THRESHOLD_MAX && (-I_THRESHOLD_MIN > error || I_THRESHOLD_MIN < error)) {
    
      if(not timerStarted){
        timerStarted = true;
        startTimer(1000);
  
              }
      I_term = I_term + (Ki * error);
      if (hasTimerExpired()) {
        I_term = 0;
        P_term = 0;
        D_term = 0;
      }
    } else
    {
      timerStarted = false;
    }
    lastTimerStarted = timerStarted;
    //else {  I_term = 0;  }
    //if( -I_THRESHOLD < error && error < I_THRESHOLD) {  I_term = 0;  }
    //else {  I_term = I_term + (Ki * error);  }



    PID_out = (P_term + I_term + D_term);
    PID_out_mapped = map(PID_out, -240, 240, SERVO_MIN, SERVO_MAX);

    PID_out_constrained = constrain(PID_out, SERVO_MIN, SERVO_MAX);

    last_time = current_time;
    last_error = error;
  }


 

   servo_pos = (int)constrain(PID_out_mapped, SERVO_MIN, SERVO_MAX);
   servo.write(servo_pos);
    



  
  
  Serial.print(setpoint_);
  Serial.print(" ");
  Serial.println(process_value);
  
  
  /*
  Serial.print(PID_out);
  Serial.print(" ");
  Serial.print(PID_out_mapped);
  Serial.print(" ");
  Serial.print(servo_pos);
  Serial.println(" ");
  */

  /*
  Serial.print(P_term);
  Serial.print(" ");
  Serial.print(I_term);
  Serial.print(" ");
  Serial.print(D_term);
  Serial.println(" ");
  */
  
  

 /*
  Serial.print(PID_setpoint);
  Serial.print(" ");
  Serial.print(distance);
  Serial.print(" ");
  Serial.print(PID_process);
  Serial.print(" ");
  Serial.println(servo_pos); 
  */

  /*
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(distance_filtered);
  */

 // while(millis() - cycle_start < MINIMUM_CYCLE_TIME){ }
}


double getDistance(int n)
{
  long sum=0;
  for(int i=0;i<n;i++)
  {
    sum=sum+analogRead(SENSOR_PIN);
  }  
  float adc=sum/n;
  double volt = (adc/1023) * 5;
  double cm = 29.998 * pow(volt, -1.173);
  return cm;
}
