/*
  Pulsed Motor Driver v0.1 by Waveguide

  This sketch turns your Arduino into a controller and monitor for 
  pulsed electric motors, like the Adams Motor or the Bedini SG.

  Using two potentiometers, you can control the duty cycle and the 
  timing of the drive pulses.

  The serial monitor also logs RPM, duty cycle, pulse duration, and 
  the timing of the pulse in degrees from the drive core center.

  Inputs:
  * D2: A3144 Hall effect sensor
  * A0: 10K potentiometer to control duty cycle
  * A1: 10K potentiometer to control pulse timing

  Outputs:
  * D8: Drive coil(s)

  Created 22/12/2020
  By Nick Kraakman
  Modified 23/12/2020
  By Nick Kraakman

  https://waveguide.blog >> ***ADD LINK TO BLOG POST HERE ***

  TODO: 
  - implement acceleration logging   
  - implement input current sensing
  - implement input voltage sensing
  - implement power logging
  - implement core temperature sensing
  - implement generator coil switching
  - implement multi-pulse option
*/


/** 
 * Variables
 */
#define NUMB_POLES    4         // Number of rotor magnet poles

#define HALL_SENSOR   2         // HALL SENSOR: Digital IN 2
#define DUTY_POT      A0        // Analog IN A0
#define DELAY_POT     A1        // Analog IN A1
#define DRIVE_COIL    8         // DRIVE COIL: Digital OUTPUT 8

static int rpm = 0;             // Moving average RPM
volatile int rpm_array[5];      // Array with 5 samples to calculate moving average RPM
static int index = 0;           // Number between 0 - 4, indicates place in rpm_array to replace

volatile float period = 0;      // Time between magnets passing by Hall sensor
volatile float last_fall = 0;   // Time of previous Hall trigger falling edge
volatile float hall_period = 0; // Time during which Hall sensor was ON, half of that is rotor magnet aligned with center of drive core
static bool hall = false;

static int duty_value = 0;      // Analog value from duty pot, 0 - 1023
static int delay_value = 0;     // analog value from delay pot, 0 - 1023
static int pulse_delay = 0;     // Pulse delay in µS after Hall interrupt, controlled by potentiometer
static int pulse_degrees = 0;   // Degrees before or after the rotor magnet aligns with drive core at which the pulse started
static int pulse_time = 0;      // Pulse duration in µS, controlled by potentiometer


/**
 * Setup code, which runs once
 */
void setup() {
  // Start serial output
  Serial.begin(9600);  // Could be set to a higher baud value if needed
  delay(1000);
  Serial.println("\n\n*** Pulsed Motor Driver v0.1 by Waveguide ***\n\n");

  // Set pin modes
  pinMode(HALL_SENSOR, INPUT_PULLUP);
  pinMode(DUTY_POT, INPUT);
  pinMode(DELAY_POT, INPUT);
  pinMode(DRIVE_COIL, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Digital pin 2 of Arduino is set as the interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), hall_trigger, CHANGE);
}


/**
 * Main code, which runs continuously
 */
void loop() 
{  
  get_pots();         // Read potentiometer values
  
  calc_rpm();         // Calculate the RPM

  //get_temp();       // Get temperature of the drive coil core

  //get_current_in(); // Get input current

  //get_bat_v();      // Get battery voltage

  send_pulse();       // Send pulse to drive coils
}


/**
 * Read and store values from the potentiometers as analog 0 - 1023 values
 */
void get_pots()
{
  duty_value = analogRead(DUTY_POT);
  delay_value = analogRead(DELAY_POT);
  delay(1);           // delay in between reads for stability
}


/** 
 * Calculate moving average RPM based on 5 samples
 * 
 * @see https://forum.arduino.cc/index.php?topic=211722.0
 */
void calc_rpm()
{  
  if(period > 10)
  {  
    // Calculate RPM
    if (index > 4)
    {
        index = 0;      // Reset index
    }
    
    rpm_array[index] = 60*(1000000/(period*NUMB_POLES));
      
    // Compute the avg rpm
    rpm = (rpm_array[0] + rpm_array[1] + rpm_array[2] + rpm_array[3] + rpm_array[4]) / 5;

    index = index + 1;  // Increment index by 1

    // Calculate acceleration
    // See: https://sciencing.com/calculate-angular-acceleration-7508269.html
    // acceleration = ((end_rpm - start_rpm) * 60)/(end_time_s - start_time_s);
  }
}


/**
 * Send pulse to the drive coils
 */
void send_pulse()
{ 
  if (hall)
  {
    // South pole passed by Hall sensor
    hall = false;                     // Reset hall variable

    pulse_delay = period * ((float) delay_value/1023);  // Delay is a % of the period, so will pulse at same point for low and high RPMs
    
    // Calculate degrees of delay from drive core center
    // Drive core center is hall_period/2
    // During period, rotor turns 360º/number of poles
    // So period/(360/NUMB_POLES) = time per degree of rotation
    // A negative value means pulsed before rotor magnet was aligned with the drive coil core
    pulse_degrees = (pulse_delay - ((float) hall_period / 2))/(period / (360 / NUMB_POLES));
    
    pulse_time = period * ((float) duty_value/1023);    // Multiply period by duty cycle to get pulse ON time
    
    delayMicroseconds(pulse_delay);   // Delay pulse ON by value from potentiometer

    digitalWrite(DRIVE_COIL, HIGH);   // Turn pulse ON
    digitalWrite(LED_BUILTIN, HIGH);  // Turn LED ON

    delayMicroseconds(pulse_time);    // Wait the duration of the pulse
      
    digitalWrite(DRIVE_COIL, LOW);    // Turn pulse OFF
    digitalWrite(LED_BUILTIN, LOW);   // Turn LED OFF

    print_data();                     // Print data to serial port each time pulse is triggered
  }
}


/**
 * Send data to serial port in CSV format
 * This data can then be plotted using SerialPlot, for example
 */
void print_data()
{
  Serial.print(rpm); 
  Serial.print(",");
  //Serial.print(current_in);
  //Serial.print(",");
  //Serial.print(core_temp);
  //Serial.print(",");
  //Serial.print(bat_v);
  //Serial.print(",");
  Serial.print(((float) duty_value/1023)*100);    // Duty cycle in %
  Serial.print(",");
  Serial.print(pulse_time);
  Serial.print(",");
  Serial.println(pulse_degrees);
}


/**
 * Fires each time a magnet passes by the Hall sensor
 */
void hall_trigger()
{
  if (PIND & (1<<PD2))  // Direct read of digital pin 2 for much faster processing than digitalRead()
  {
    // Rising edge
    hall_period = (micros() - last_fall);
  } else {
    // Falling edge
    period = (micros() - last_fall);
    last_fall = micros();
    hall = true;
  }
}
