#include <Enes100.h>
//constants needed for turning
const float Pi = 3.14;
const float halfPi = 1.57;
const float tolerance = 0.10;

//thermistor variables
#define ntc_pin0 A0
float temperature0;
#define ntc_pin1 A1
float temperature1;
#define ntc_pin2 A2
float temperature2;
#define ntc_pin3 A3
float temperature3;
#define vd_power_pin 2
#define nominal_resistance 10000  // Nominal resistance at 25⁰C
#define nominal_temeprature 25    // temperature for nominal resistance (almost always 25⁰ C)
#define samplingrate 5            // Number of samples
#define beta 3950                 // The beta coefficient or the B value of the thermistor (usually 3000-4000)
#define Rref 10000                // Value of  resistor used for the voltage divider
int samples = 0;                  // array to store the samples
int fires = 0;

//motor declarations
// Arm motor connections
#define PIN_A 30
#define PIN_B 31
//front
// Motor A connections
#define enA 9
#define in1 8
#define in2 7

// Motor B connections
#define enB 3
#define in3 5
#define in4 4

//rear
//Motor C connections
#define enC 10
#define in5 13
#define in6 6

//Motor D connections
#define enD 11
#define in7 2
#define in8 12

//left HC-SR04 pins and variables
#define trigPin 45
#define leftEchoPin 44
long leftDuration;
int leftDistance;

//right HC-SR04 pins
#define rightEchoPin 46
long rightDuration;
int rightDistance;


//other variables
boolean missionCompleted = false;
boolean inMissionArea = false;

void setup() {
  //wifi module
  delay(200);
  Enes100.begin("Calcifer's Minions", FIRE, 13, 50, 51);  //number meaning (aruco id, tx, rx)
  delay(200);
  Enes100.println("Connected!");

  //motors
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);

  //ultrasonic distance sensors
  pinMode(trigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightEchoPin, INPUT);

  Serial.begin(9600);
}

void loop() {
  //navigating to and completing mission
  if (Enes100.missionSite.y < 1 && missionCompleted == false) {
     turn(-halfPi);
     backward();
     delay(100);
     stop();
     forward();
     leftUltraSonic();
     while (leftDistance < 25) {
       stop();
       mission();
       backward();
       if (leftDistance > 25) {
         stop();
       }
       break;
     }
   }

   if (Enes100.missionSite.y > 1 && missionCompleted == false) {
     turn(halfPi);
     backward();
     delay(100);
     stop();
     forward();
     leftUltraSonic();
     while (leftDistance < 25) {
       stop();
       mission();
       backward();
       if (leftDistance > 25) {
         stop();
       }
       break;
     }
   }
  

   if (missionCompleted == true) {
     Enes100.updateLocation();
     backward();
     turn(0);
     stop();
     }
    
     //obstacle on bottom half of arena
     while ((leftDistance < 25) && (Enes100.location.y < 1)) {
       Enes100.println("obstacle detected");
       Enes100.updateLocation();
       stop();
       delay(1000);
       leftTurn();
       forward();
       delay(1000);
       rightTurn();
       leftUltraSonic();
       break;
     }

     //obstacle on top half of arena
     while (leftDistance < 25 && Enes100.location.y > 1) {
       Enes100.println("obstacle detected");
       Enes100.updateLocation();
       stop();
       delay(1000);
       rightTurn();
       forward();
       delay(1000);
       leftTurn();
       leftUltraSonic();
       break;
     }
     //entering into the end zone
     if (Enes100.location.x >= 2.8 && Enes100.location.y < 1) {
        forward();
       if (Enes100.location.y > 1) {
         stop();
         rightTurn();
         forward();
         if(Enes100.location.y < 0.8){
           leftTurn();
           stop();
           forward();
         }

        if (Enes100.location.x == 3.8) {
           stop();
         }
       }
     }


     while (leftDistance < 25 && Enes100.location.y > 1) {
       Enes100.println("obstacle detected");
       Enes100.updateLocation();
       stop();
       delay(1000);
       leftTurn();
       forward();
       delay(1000);
       rightTurn();
       leftUltraSonic();
       break;
     }
   }



}

void mission() {
  leftUltraSonic();
  delay(1000);
  rightUltraSonic();


  if (leftDistance == rightDistance) {
    Enes100.mission(TOPOGRAPHY, TOP_C);
    Serial.println("C");
  } else if (leftDistance > rightDistance) {
    Enes100.mission(TOPOGRAPHY, TOP_B);
    Serial.println("B");
  } else {
    Enes100.mission(TOPOGRAPHY, TOP_A);
    Serial.println("A");
  }

  delay(2000);

  armDown();

  thermistor0();
  thermistor1();
  thermistor2();
  thermistor3();

  Enes100.mission(NUM_CANDLES, fires);
  Serial.println("Num of fires:");
  Serial.println(fires);


  delay(2500);

  armUp();

  missionCompleted = true;
}

//methods for OTV motion
void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
}

void forward() {
  analogWrite(enA, 180);
  analogWrite(enB, 160);
  analogWrite(enC, 180);
  analogWrite(enD, 160);

  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}

void backward() {
  analogWrite(enA, 180);
  analogWrite(enB, 180);
  analogWrite(enC, 180);
  analogWrite(enD, 180);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}

void turn(float target) {
  analogWrite(enA, 65);
  analogWrite(enB, 65);
  analogWrite(enC, 65);
  analogWrite(enD, 65);
  

  while (!(abs(Enes100.location.theta - target) < tolerance)) {
    Enes100.updateLocation();
    Enes100.print(Enes100.location.theta);

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    digitalWrite(in5, HIGH);
    digitalWrite(in6, LOW);

    digitalWrite(in7, LOW);
    digitalWrite(in8, HIGH);
    Enes100.println(" now spinning");
  }
  stop();
}

//methods for the OTV's arm movement
void armUp() {
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, LOW);
  delay(1000);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
}

void armDown() {
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, HIGH);
  delay(1350);
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
}


//methods for sensors on OTV
void leftUltraSonic() {
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  leftDuration = pulseIn(leftEchoPin, HIGH);
  leftDistance = leftDuration * 0.034 / 2;

  Serial.print("Distance from Left Sensor: ");
  Serial.println(leftDistance);
}

void rightUltraSonic() {
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  rightDuration = pulseIn(rightEchoPin, HIGH);
  rightDistance = rightDuration * 0.034 / 2;

  Serial.print("Distance from Right Sensor: ");
  Serial.println(rightDistance);
}

void thermistor0() {
  uint8_t i;
  float average;
  samples = 0;
  // take voltage readings from the voltage divider
  digitalWrite(vd_power_pin, HIGH);
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead(ntc_pin0);
    delay(10);
  }
  digitalWrite(vd_power_pin, LOW);
  average = 0;
  average = samples / samplingrate;
  Serial.println("\n \n");
  Serial.print("ADC readings ");
  Serial.println(average);

  // Calculate NTC resistance
  average = 1023 / average - 1;
  average = Rref / average;
  Serial.print("Thermistor resistance ");
  Serial.println(average);

  temperature0 = average / nominal_resistance;           // (R/Ro)
  temperature0 = log(temperature0);                      // ln(R/Ro)
  temperature0 /= beta;                                  // 1/B * ln(R/Ro)
  temperature0 += 1.0 / (nominal_temeprature + 273.15);  // + (1/To)
  temperature0 = 1.0 / temperature0;                     // Invert
  temperature0 -= 273.15;                                // convert absolute temp to C

  Serial.print("Temperature @ Pos 0: ");
  Serial.print(temperature0);
  Serial.println(" *C");

  if (temperature0 > 35) {
    fires += 1;
  }

  delay(2000);
}

void thermistor1() {
  uint8_t i;
  float average;
  samples = 0;
  // take voltage readings from the voltage divider
  digitalWrite(vd_power_pin, HIGH);
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead(ntc_pin1);
    delay(10);
  }
  digitalWrite(vd_power_pin, LOW);
  average = 0;
  average = samples / samplingrate;
  Serial.println("\n \n");
  Serial.print("ADC readings ");
  Serial.println(average);

  // Calculate NTC resistance
  average = 1023 / average - 1;
  average = Rref / average;
  Serial.print("Thermistor resistance ");
  Serial.println(average);

  float temperature1;
  temperature1 = average / nominal_resistance;           // (R/Ro)
  temperature1 = log(temperature1);                      // ln(R/Ro)
  temperature1 /= beta;                                  // 1/B * ln(R/Ro)
  temperature1 += 1.0 / (nominal_temeprature + 273.15);  // + (1/To)
  temperature1 = 1.0 / temperature1;                     // Invert
  temperature1 -= 273.15;                                // convert absolute temp to C

  Serial.print("Temperature @ Pos 1: ");
  Serial.print(temperature1);
  Serial.println(" *C");

  if (temperature1 > 35) {
    fires += 1;
  }

  delay(2000);
}

void thermistor2() {
  uint8_t i;
  float average;
  samples = 0;
  // take voltage readings from the voltage divider
  digitalWrite(vd_power_pin, HIGH);
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead(ntc_pin2);
    delay(10);
  }
  digitalWrite(vd_power_pin, LOW);
  average = 0;
  average = samples / samplingrate;
  Serial.println("\n \n");
  Serial.print("ADC readings ");
  Serial.println(average);

  // Calculate NTC resistance
  average = 1023 / average - 1;
  average = Rref / average;
  Serial.print("Thermistor resistance ");
  Serial.println(average);

  float temperature2;
  temperature2 = average / nominal_resistance;           // (R/Ro)
  temperature2 = log(temperature2);                      // ln(R/Ro)
  temperature2 /= beta;                                  // 1/B * ln(R/Ro)
  temperature2 += 1.0 / (nominal_temeprature + 273.15);  // + (1/To)
  temperature2 = 1.0 / temperature2;                     // Invert
  temperature2 -= 273.15;                                // convert absolute temp to C

  Serial.print("Temperature @ Pos 2: ");
  Serial.print(temperature2);
  Serial.println(" *C");

  if (temperature2 > 35) {
    fires += 1;
  }

  delay(2000);
}

void thermistor3() {
  uint8_t i;
  float average;
  samples = 0;
  // take voltage readings from the voltage divider
  digitalWrite(vd_power_pin, HIGH);
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead(ntc_pin3);
    delay(10);
  }
  digitalWrite(vd_power_pin, LOW);
  average = 0;
  average = samples / samplingrate;
  Serial.println("\n \n");
  Serial.print("ADC readings ");
  Serial.println(average);

  // Calculate NTC resistance
  average = 1023 / average - 1;
  average = Rref / average;
  Serial.print("Thermistor resistance ");
  Serial.println(average);

  temperature3 = average / nominal_resistance;           // (R/Ro)
  temperature3 = log(temperature3);                      // ln(R/Ro)
  temperature3 /= beta;                                  // 1/B * ln(R/Ro)
  temperature3 += 1.0 / (nominal_temeprature + 273.15);  // + (1/To)
  temperature3 = 1.0 / temperature3;                     // Invert
  temperature3 -= 273.15;                                // convert absolute temp to C

  Serial.print("Temperature @ Pos 3: ");
  Serial.print(temperature3);
  Serial.println(" *C");

  if (temperature3 > 35) {
    fires += 1;
  }

  delay(2000);
}



void rightTurn() {
  stop();
  delay(1000);

  analogWrite(enA, 180);
  analogWrite(enB, 180);
  analogWrite(enC, 180);
  analogWrite(enD, 180);


  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);

  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);

  delay(950);
  stop();
}

void leftTurn() {
  stop();
  delay(1000);

  analogWrite(enA, 180);
  analogWrite(enB, 180);
  analogWrite(enC, 180);
  analogWrite(enD, 180);


  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);

  digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);

  delay(1100);
  stop();
}
