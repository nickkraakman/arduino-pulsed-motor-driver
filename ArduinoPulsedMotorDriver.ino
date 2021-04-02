/*
  Pulsed Motor Driver v0.3 by Waveguide

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
  * A2: Voltage divider to measure battery voltage (1M & 100K 1% resistors for up to 55V)

  Outputs:
  * D8: Base of MJL21194 or similar NPN transistor, which powers the 
        drive coil(s) directly, or a 2N2222 transistor to drive a PNP     
        high-side switch

  Created 22/12/2020
  By Nick Kraakman
  Modified 05/03/2021
  By Nick Kraakman

  https://waveguide.blog/adams-motor-generator/

  TODO:   
  - implement input current sensing
  - implement power logging
  - implement core temperature sensing
  - implement generator coil switching
*/


/** 
 * Variables
 */
#define NUMB_POLES    4         // Number of rotor magnet poles
#define SAMPLES       8         // Number of samples used to smooth the period measurement
#define PULSES        1         // Number of pulses per trigger

#define HALL_SENSOR   2         // HALL SENSOR: Digital IN 2
#define DUTY_POT      A0        // Analog IN A0
#define DELAY_POT     A1        // Analog IN A1
#define V_SENSOR      A2        // Analog IN A2
#define DRIVE_COIL    9         // DRIVE COIL: Digital OUTPUT 9

static float rpm = 0.0;         // RPM of the motor

volatile unsigned long now = 0;               // Current time in µS
volatile unsigned long period = 0;            // Time between magnets passing by Hall sensor
volatile unsigned long last_fall = micros();  // Time of previous Hall trigger falling edge, init as micros() instead of 0, else first calculated period is way too large, causing trouble with starting the motor
volatile unsigned long hall_period = 0;       // Time during which Hall sensor was ON, half of that is rotor magnet aligned with center of drive core
volatile bool hall = false;                   // True if Hall sensor has been triggered, false once pulse completes
volatile unsigned long periods[SAMPLES];      // Moving average period using several samples
volatile bool periods_full = false;           // True if the periods array has been filled with samples
volatile byte index = 0;                      // Indicates current place in period_array
volatile unsigned long sum = 0;               // Running total of period samples

static bool pulsing = false;                  // True while we're pulsing the drive coils
static bool high = false;                     // True if drive coil pin is HIGH, false if LOW

static bool get_voltage = true;         // True if a new voltage measurement needs to be made
static int voltage_value = 0;           // Analog value from voltage divider, 0 - 1023
static float voltage = 0.0;             // Measured voltage on V_SENSOR pin, 0 - 5V

static int duty_value = 0;              // Analog value from duty pot, 0 - 1023
static int delay_value = 0;             // analog value from delay pot, 0 - 1023
static unsigned long pulse_delay = 0;   // Pulse delay in µS after Hall interrupt, controlled by potentiometer
static int pulse_degrees = 0;           // Degrees before or after the rotor magnet aligns with drive core at which the pulse started
static unsigned long pulse_time = 0;    // Pulse duration in µS, controlled by potentiometer

static unsigned long last_print = 0;    // Time of previous print to serial monitor in mS
static unsigned long last_read = 0;     // Time of previous analogRead of potentiometers in mS


/**
 * Setup code, which runs once
 */
void setup() {
  // Start serial output
  Serial.begin(9600);  // Set baud rate
  delay(1000);
  Serial.println("\n\n*** Pulsed Motor Driver v0.3 by Waveguide ***\n\n");

  // Set pin modes
  pinMode(HALL_SENSOR, INPUT_PULLUP);
  pinMode(DUTY_POT, INPUT);
  pinMode(DELAY_POT, INPUT);
  pinMode(V_SENSOR, INPUT);
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
  now = micros();

  if (!pulsing)
  {
    // Stuff to do while we're NOT pulsing the coils,
    // like performing calculations and printing data
    read_analog();      // Read potentiometer values and battery voltage
    print_data();       // Log data to serial monitor every second
  } 

  if (hall) 
  {
    send_pulse();       // Send pulse to drive coils
  }
}


/**
 * Read and store values from the potentiometers and voltage divider as analog 0 - 1023 values
 */
void read_analog()
{
  // Read only when NOT pulsing, and delay between reads for stability
  if (millis() - last_read > 10)
  {
    last_read = millis();

    // We set 50% duty value if RPM is low to help get the motor started
    if (rpm < 300)
    {
      duty_value = 512;
    } else {
      duty_value = analogRead(DUTY_POT);
    }
    
    delay_value = analogRead(DELAY_POT);

    // We only need to get a voltage reading once every print cycle, saves ~240µS on other cycles
    if (get_voltage)
    {
      // Apparently we need to read an analog pin TWICE and discard the first read to increase the odds of a correct reading...
      analogRead(V_SENSOR); voltage_value = analogRead(V_SENSOR);
      get_voltage = false;
    }

    // Calculate pulse delay and pulse time
    // We divide by 2048 instead of 1024, because we only want a max of 1/2 period of delay, and max 50% duty
    // This also gives our potentiometers more resolution and thus allows for finer adjustments and smaller unwanted variations
    pulse_delay = (period * delay_value) / 2048;    // Delay is a % of the period, so will pulse at same point for low and high RPMs

    pulse_time = (period * duty_value) / 1024;      // Multiply period by duty cycle to get pulse ON time
  }           
}


/**
 * Send pulse to the drive coils
 */
void send_pulse()
{ 
  // Start pulsing after delay in non-blocking way
  if (!high && now - last_fall >= pulse_delay)
  {
    pulsing = true;
    
    if (PULSES > 1)
    {
      // Calculate new pulse time if we're using multiple pulses per trigger
      pulse_time = pulse_time / (PULSES + PULSES - 1);
    }
    
    // Once we're pulsing, that's all we want to do, so we can use blocking delays (pfew!)
    for (int i = 0; i < PULSES; i++) 
    {
      //digitalWrite(DRIVE_COIL, HIGH);     // Turn pulse ON
      PORTB |= (1<<PB1);                    // Set digital port 9 HIGH directly, digitalWrite too slow
      //digitalWrite(LED_BUILTIN, HIGH);    // Turn LED ON
      high = true;

      delayMicroseconds(pulse_time);

      //digitalWrite(DRIVE_COIL, LOW);      // Turn pulse OFF
      PORTB &= ~(1<<PB1);                   // Set digital port 9 LOW directly, digitalWrite too slow
      //digitalWrite(LED_BUILTIN, LOW);     // Turn LED OFF
      high = false;

      // If there is another pulse coming, stay low until next loop
      if (i + 1 < PULSES)
      {
        delayMicroseconds(pulse_time);
      }
    }

    pulsing = false;
    hall = false;                           // We're done pulsing, reset hall variable
  }
}


/**
 * Send data to serial port in CSV format
 * This data can then be plotted using SerialPlot, for example
 * We calculate a few things here that are only needed for printing, 
 * not for controlling the motor
 */
void print_data()
{
  if (millis() - last_print > 1000)
  {
    // Calculate degrees of delay from drive core center
    // Drive core center is hall_period/2
    // During period, rotor turns 360º/number of poles
    // So period/(360/NUMB_POLES) = time per degree of rotation
    // A negative value means pulsed before rotor magnet was aligned with the drive coil core
    pulse_degrees = (pulse_delay - ((float) hall_period / 2))/(period / (360 / NUMB_POLES));

    rpm = 60*((float) 1000000/(period*NUMB_POLES));
    
    Serial.print((int) rpm); // RPM
    Serial.print(",");
    Serial.print(((float) duty_value/1024)*100);            // Duty cycle in %
    Serial.print(",");
    Serial.print(pulse_time);
    Serial.print(",");
    Serial.print(pulse_delay);
    Serial.print(",");
    Serial.print(period);
    Serial.print(",");
    Serial.print(pulse_degrees);
    Serial.print(",");

    // Source voltage, 
    // 5 = reference voltage, 4.857 = voltage divider multiplier, adjust to your own situation
    // See: https://startingelectronics.org/articles/arduino/measuring-voltage-with-arduino/
    Serial.println(((voltage_value * 5.0) / 1024.0) * 4.857);

    get_voltage = true;  // Now that we printed the voltage, we want to get a new voltage reading
    
    last_print = millis();
  }
}


/**
 * Fires each time a magnet passes by the Hall sensor
 */
void hall_trigger()
{
  if (PIND & (1<<PD2))  // Direct read of digital pin 2 for much faster processing than digitalRead()
  {
    // Rising edge
    hall_period = now - last_fall;
  } else {
    // Falling edge
    
    // Calculate moving average period (only the necessary part, rest happens in calculate_period() function to save processing time)
    sum = sum - periods[index];         // Subtract last period
    periods[index] = now - last_fall;   // Calculate period and store in array
    last_fall = now;                    // Set time of this trigger for the next trigger

    sum = sum + periods[index];         // Add current period

    index = index + 1;                  // Move pointer to next position in index
  
    // If we're at the end of the array...
    if (index >= SAMPLES) {
      periods_full = true;
      index = 0;                        // Reset index
    }
  
    if (periods_full) {
      period = sum / SAMPLES;           // Calculate the moving average
    } else {
      period = periods[index - 1];      // Until we have filled our array to calculate a running average, use the raw readings
    }
    
    hall = true;
  }
}
