/***************** L16A_Phenotype_10Feb2017 *****************
 *  Created 10 February, 2017
 *  Author: Nick Livingston (nilivingston@vassar.edu)
 *  
 *  As the name suggests, the purpose of this program is to
 *  instantiate the phenotype of an individual Landro, L16A
 *  agent for the evolutionary experiments to be performed
 *  using the L16A platform.  This .ino file expects a header
 *  file, which an evolutionary algorithm specifies for each
 *  individual within a population according to L16A's 
 *  genotype to phenotype (GtoP) mapping algorithm.
 *  
 *  A SoftwareSerial library is included in order to allow for
 *  the serial communication between the Arduino Mega 2560, and
 *  the iRobot Create 2.
 *  
 *  The SPI library is needed for the Catalex Micro SD Card
 *  Adapter.  The SD library contains functions which allow for
 *  files to be created, opened, read, written to, and closed
 *  on a micro SD card that is plugged into the Catalex adapter.
 *  
 *  The setup() function initailizes communication with the SD
 *  card adapter, as well as with the Create 2.  It also
 *  initializes the artificial neural network (ANN) specified
 *  by the axiliary header file.  Implicit in the structure of
 *  this network is the specification of which of L16A's sixteen
 *  sensors is to be used for this individual agent.  All
 *  sensors are permanently mounted to the chassis of L16A, and
 *  plugged into analog ports in an order that reflects the
 *  clockwise arrangment of sensors.  An individual's header file
 *  specifies which of these sensors is "grown" in that individual's
 *  development stage, as well as how it connects to the Landro's
 *  brain -- hidden and or motor (output) neurons.
 *  
 *  The loop() function calls upon all other auxiliar functions
 *  to generate an agent's behavior.  This behavior includes querying
 *  all sensor ports for a given number of samples, sending the
 *  average of those samples to the ANN via the appropriate input
 *  neurons, and interpreting ANN output data into a form that can
 *  be sent to the function for controlling the Create 2 motors.  Thus,
 *  the loop() function facilitates a Landro's sense, think, act cylce.
************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h> 

/*The baud rate change (BRC, or "baud" for short) is
 *a digitial pin that can be used for telling the iRobot
 *Create 2 to change the baud rate at which it communicates.
 *The Create 2's default baud rate is expecting a mini DIN
 *to USB connection with a full sized computer (e.g. PC).
 *Since we are communicating via an Arduino Mega 2560, we
 *want to change this baud rate to be more Arduino friendly.
 *The baud rate of choice shall be 19200, which for simplicity
 *shall also be used to perform actions when interfacing with
 *the micro SD shield.
*/
#define baudPin  11
/*There is a momentary button soldered onto L16A's breakout
 * board.  This button is wired into a digital port.  It can
 * be plugged into any digital port, the one below was
 * convenient at the time of writing this program.
 */
#define contPin  2
/* The chip select pin is a reserved pin on Arduinos
 * (pin 53 for the Mega 2560), which in this context is
 * used to initialize the Catalex SD Card Adapter.
 */
#define chipSelect 53

#define NUM_PORTS 16
/* STANDARDIZE FORMAT....
 *  ports specifies all of the ports available to L16A.
 *  All prots are specified because the fitness function
 *  requires input from all sensors, even if the ANN
 *  only uses a subset.
 */
int ports[NUM_PORTS];
/**********************************************************/
/* This function runs through every available sensor, and
 * obtains the value for that sensor for the current timestep. 
 * That value gets stored in the appropriate input node of the
 * ANN.  In essence, sense() tells the ANN about the world on a
 * given iteration.
 */
void sense();
/**********************************************************/
/* This function executes the computations of L16A's ANN.
 */
 void neural_net();
 /**************************************************************/
/* This function implements the activation function for updating
 * network nodes(specifically hidden nodes).  There are two
 * slightly different formulations, one for recurrent connections
 * and one for non-recurrent connections.
 */
void activation();
/**********************************************************/
/*This function records all experimentally relevant data to
 *a micro SD card.  These are the data that will be used
 *to analyze the experiment, and to compute an individual's
 *fitness.
 */
void record();
/**********************************************************/
/*This function takes two values which should range between
 * -500 and 500 (ostensibly with units of mm/sec) and sends
 * those values to the Create 2 as a sequence of four bytes
 * (two per motor), which instruct the Create 2 motors how
 * to drive.  Negative values for rightValue and leftValue
 * cause the motors to spin backward.  Larger values denote
 * faster spin.
 */
void driveMotors(int rightValue, int leftValue);
/**********************************************************/
/*This currently does not work reliably, but it spurpose is to
 * poll the front bumpers of the Create 2.  Which is to say,
 * factory made sensors, not ones installed after the fact.
 * It is somewhat unclear as to what the correct command
 * codes for reading the bumpers are, and what the proper way is
 * to specify what data subset should be read, or how it should
 * be read.
 */
bool checkBumpSensors();
/**********************************************************/
//Limits speed to a range between -500 and 500 by clipping it.
int checkSpeed(int spd);
  

//https://cdn-shop.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf
/**********************************************************/
void setup(){
  //Give the iRobot Create 2 a chance to "wake up".
  delay(2000);
  //Open communication with iCreate
  pinMode(baudPin, OUTPUT);
  //Set data rate for the SoftwareSerial port, this is the iRobot's default
  Serial3.begin(19200);
  //Send three low signal pulses to the BRC pin to enact the baud rate
  //change that is specified above.
  digitalWrite(baudPin, LOW);
  delay(500);
  digitalWrite(baudPin, LOW);
  delay(500);
  digitalWrite(baudPin, LOW);
  delay(500);
  
  //Start robot in safe mode
  Serial3.write(128);//128 is the "start" code.
  Serial3.write(131);//Safe = 131, Full = 132
  delay(1000);
  // Open serial communications and wait for port to open:
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");
    
  //See if the SD card is present and can be initialized:
  while (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
  }
  Serial.println("card initialized.");

  //NBL: Might not use this during evolutionary experiments.
  //But may be a way to transition from trial to trial or something.
  //Whether for a single individual, or between individuals.
  pinMode(contPin, INPUT);

  //Specify ports for L16A:
  ports[NUM_PORTS] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

  //Initialize sensor values
  for(int i = 0; i < numPorts; i++){
    for(int j = 0; j < numSensorSamples; j++){
      sensorValues[i][j] = 0;
    }
  }

}

/***************************************************************/
//This is the main loop of arduino code and calls everything else
// if there are issues something may be commented out here

void loop(){
  
    int i;
    
    unsigned long endTime = 0;
    unsigned long startTime = millis();

    //For debugging
    //numOrientations = 0;
    for(i = 0; i < numPorts; i ++){
      Serial.println("Here we go again");
//      act();
//      driveDirect(0, 0);
      sense();
      record();
//      phi += phiIncrement;
//      delay(senseThinkTime);
     //actTime += 10;
    }
    //Reset orientation
//   phi = 0;
    //endTime = millis();
    //trialTime = endTime - startTime;
    //actTime = 12950/16;

      //Leave this out for debugging for now.
      //recordDataSet();
//      endTest(1);
    
    //We have come to the end of the trial.  A song will play on loop until the
    //front bumper is pressed.  You with then have 1 second to get out of the way.
    //After the bumper is pressed, the bot will drive forward 30 cm to the next 
    //trial position.
    //End rotations... signal transition (drive straight to next (x, y)).
    //endTest(0);
    //Drive to next (x, y )
    delay(1000);
 //   transition();
    //driveDirect(0, 0);
    //delay(5000);
    //Signal end of getting to next (x, y).  Press bumper to start next set of rotations
//    endTest(0);
}

/***************************************************************/
/* This function uses local variables to loop through L16A's sensors,
 * and store their current values in the "sensorValues" array.  The
 * matrix "analogPorts" names, in sequence, all 16 analog sensor ports
 * (alternating IR and LDR in a clockwise fashion, starting with the
 * former).  The port numbers correspond to the sensor positions, starting with
 * IR1 (which in broader terms is sensor 0), positioned at the exact front of
 * Landro in line with the Create 2 IR beacon sensor.  Moving clockwise, sensor 1
 * is LDR1, which sits immediately to the right of IR1 and plugs into port A1.
 * The pattern continues, and terminates with LDR8 -- sensor 15 -- in port A15.
 * 
 * Each sensor is read 10 time in succession.  The average of these readings
 * will be taken to be "the" sensor value for the current iteration of
 * behavior.  This averaging is done elsewhere.  Though, sensorValues could
 * simply become a one dimensional array, each cell of which is first populated
 * by summing, and then outside of the "j" loop, averaged.
 */
void sense(){
  int i, j;
  
  for(i = 0; i < numPorts; i++){
    for(j = 0; j < numSensorSamples; j++){
      sensorValues[i][j] = analogRead(analogPorts[i]);      
    }
  }

}
/***************************************************************/
/* This function executes L16A's ANN computations.  This handles
 *  all updates downstream of the inputs array.
 */
void neural_net(){
  int h, p, o;
  
  //Update hidden nodes using inputs for time t
  for(h = 0; h < NUM_HIDDEN; h++){
    hidden[h] = 0;
    for(i = 0; i < NUM_INPUT; i++) {   
      hidden[h] = hidden[h] + input[i] * input_to_hidden[i][h];
    }
    //Update hidden nodes using hidden (last) values) from time t-1
    for(p = 0; p < NUM_HIDDEN; p++){
      hidden[h] = hidden[h] + old_hidden[p] * hidden_to_hidden[p][h];
    }
  }
  
  for(h = 0; h < NUM_HIDDEN; h++){
    //Apply tanh function to total update
    hidden[h] = activation(hidden[h]);
    
    //Save the hidden nodes state for the future use
    old_hidden[h] = hidden[h];
  }
  //Update output (motor) nodes based on hidden nodes 
  for(o = 0; o < NUM_OUTPUT; o++){
    for(h = 0; h < NUM_HIDDEN; h++){
      output[o] = output[o] + hidden[h] * hidden_to_output[h][o];
    }
    //Apply tanh equation to total update
    output[o] = 0.5 + 0.5 * activation(output[o]);
}
/**************************************************************/
/* This function implements the activation function for updating
 * network nodes(specifically hidden nodes).  There are two
 * slightly different formulations, one for recurrent connections
 * and one for non-recurrent connections.
 */
int activation(int value){
  int update_value;
  update_value = tanh(value);
  return update_value;
}
  
}
/***************************************************************/
/* This function writes trial information to a micro SD card.
 *  At this point, it is unclear what should be written.
 *  At the bare minimum, it should be ANN inputs and outputs.
 *  There should probably be some sort of time stamp.  Potentially,
 *  all sensors, whether used in the current agent's morphology
 *  or not, should be recorded for use in calculating fitness.
 *  Alternatively, fitness could be calculated on the fly, and
 *  piecewise fitness recorded.  That could be interesting.
 *  Then a running talley could be kept, but performance over time
 *  could also be plotted and analyzed.
 */
void record(){
 
}
/**********************************************************/
/*Sends two bytes per motor.  The bytes cannot be written
 * as one continuous string, so they are broken up and
 * sent in serial.  Before sending the values, however, make
 * sure they fall between -500 and 500 by calling checkSpeed().
 */
void driveMotoros(int rightValue, int leftValue){
  checkSpeed(rightValue);
  checkSpeed(leftValue);
  Serial3.write(145);//Creat 2's motor command code.
  Serial3.write(highByte(rightValue));//High and low byte for the right motor.
  Serial3.write(lowByte(rightValue));
  Serial3.write(highByte(leftValue));//High and low byte for the left motor.
  Serial3.write(lowByte(leftValue));
}

//This is somewhat complicated and I don't know exactly how it all works
// but it does so that's pretty good. It just reads the byte stream basically
// after asking for the info
//Originally sourced from http://web.ics.purdue.edu/~fwinkler/AD61600_S14/AD61600_Arduino_iRobot.pdf

/*Bumps and Wheel Drops Packet ID: 7 Data Bytes: 1, unsigned
The state of the bumper (0 = no bump, 1 = bump) and wheel drop sensors (0 = wheel raised, 1 = wheel
dropped) are sent as individual bits. */
bool checkBumpSensors() {
  int* buffer;
  bool ret = false;
  byte msb = 0;
  byte lsb = 0;
  Serial3.write(142);
  Serial3.write(7);

  if(Serial1.available() > 0){
      msb = Serial1.read();
      lsb = Serial1.read();
      *buffer = (msb << 7) | lsb;
      ret = true;
  }
  return ret;
}
/**************************************************************/
/* This funciton pimits speed to a range between -500 and 500 
 * by clipping it.  Speed (spd) is passed by reference, so there 
 * is no need to have an explicit return value.
 */
void checkSpeed(int &spd){
 if(spd > 500){
  spd = 500;
 }
 else if(spd < -500){
  spd = -500;
 }
}
  
    

