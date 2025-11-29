//168 square 169 sine
// 0.175 0.0008 0.002 sqaure 10.0 0.02-0.03 0.3-0.35 sine

// 0.1) Pin Setup - ต่อที่พินไหน
const int Pin_Button = 11;

const int Pin_IN1 = 9;
const int Pin_IN2 = 10;

const int encoderA = 2;
const int encoderB = 3;
// 0.2) Button, Control Setup
const int CPR = 448;
const int count_Mode = 4;
volatile long encoder_count = 0;
volatile int last_Encoded = 0;
float angle = 0.0;
int encoderAvalue = 0;
int encoderBvalue = 0;


// State -> Button - เอาไว้เก็บว่า button เป็น high หรือ low
int last_Button_State = LOW;            // Button State (Last Iteration)
int current_Button_State = LOW;         // Button State (Current Iteration)

// State -> Control Enable or not ? - ให้ control loop ทำงานหรือไม่
bool control_Enabled = false;           // State to determine whether control is turned

// State -> In delay duration or not? - ตอนนี้อยู่ในช่วง delay รึเปล่า
bool delay_Active = false;              // State to determine whether it is in delay duration before the control start

// Associated time when press is occured
unsigned long last_debounce_time = 0;         // Time when last debounce occurs
const unsigned long debounce_Delay = 50;      // 50 ms 

//เก็บเวลา timer
unsigned long t_start = 0;
unsigned long activation_Time = 0;
unsigned long t_sec = 0;
const unsigned long delay_Before_Start = 5000;     // 5 seconds delay after button press to active Control
float kp = 0.188;
float ki = 0.002;
float kd = 0.0035;

float integral = 0;
float prev_error = 0;
unsigned long prev_time = 0;

const float ref_Amplitude = 180.0;
const float ref_Frequency = 0.4;
const float ref_PhaseShift = 0.0;
unsigned long lastPrintTime = 0;
const unsigned long plotInterval = 50;

// Void Setup ************************************************

void setup() {
  // setup code here, to run once:

  // Pin Setup
  pinMode(Pin_Button, INPUT);   // Button, setup pinMode

  pinMode(Pin_IN1,OUTPUT); 
  pinMode(Pin_IN2,OUTPUT); 

  // Set Serial
  Serial.begin(115200);
  delay(1000);   //delay 1000 ms = 1s, delay(unsigned long ms);

  //interrupt Function Setup
  if (count_Mode == 4){
    attachInterrupt( digitalPinToInterrupt(encoderA),ISR_4X,CHANGE);
    attachInterrupt( digitalPinToInterrupt(encoderB),ISR_4X,CHANGE);
    int MSB = digitalRead(encoderA);
    int LSB = digitalRead(encoderB);
    last_Encoded = (MSB << 1) | LSB;
  }

  last_debounce_time = millis();
  t_start = millis();

  // Print Setup Complete
  Serial.println("Setup is Completed, now running void loop");
}

// Main Loop  ***********************************************************************************************
void loop() {
  // main code here, to run repeatedly:

  unsigned long t_now = millis();  // get current time stamp
  encoderAvalue = digitalRead(encoderA);
  encoderBvalue = digitalRead(encoderB);
  // Main Loop, Part 1) Button Debounce & Toggle Function
  // - Detects when the button is "Actual" pressed (no false triggers from noise).
  // - If the system is "Off", pressing the button --> starts 5 s (delay_Before_Start = 5000) countdown before enabling control.
  // - If the system is "On", pressing the button --> immediately stops control and sets the PWM output to zero.

  // Step-by-Step in Part 1)
  // 1.1 Read the current button state
  int reading = digitalRead(Pin_Button);  // Initially reading is "0". When press --> reading is "1" (Pull Down Resistor Case, ref: https://docs.arduino.cc/built-in-examples/digital/Button/)

  // 1.2 Check if the button’s signal has changed (Pressed or Released)
  // Button state changed = mark the time for debounce check
  if (reading != last_Button_State) {  // if current button state has changed (current state != last state)
    last_debounce_time = t_now;        // Update last_Debounce_Time at this time instance
  }

  // 1.3 When press occurs, then update relevant states.
  // current_Button_State, last_Button_State, control_Enabled, delay_Active, activation_Time
  if ((t_now - last_debounce_time) > debounce_Delay) { // When time difference between (now & last press) > debounce Delay --> update key states

    if (reading != current_Button_State) { // When current button state != current_Button_State from last iteration (or it's previous button state)
      current_Button_State = reading;      // Update current_Button_State to current state reading

      if (current_Button_State == HIGH) {  // When press is occured, current_Button_State == HIGH (Pull Down Resistor Case)

        if (!control_Enabled && !delay_Active) { // Given button pressed is True --> when control has not started yet and not in delay period prior to control will active
          // Start delay before enabling control
          delay_Active = true;               // That means it's in the delay duration now e.g. 5 s before the control will start
          activation_Time = t_now;           // activation_Time records this activation event
          Serial.println("🔵 Button pressed → Starting 5s delay..."); // Let's print what's just happened
        }
        else if (control_Enabled && !delay_Active) {
          // Stop control immediately and zero PWM
          control_Enabled = false;           // Disable control
          Serial.println("🟡 Control stopped, PWM set to 0"); // Let's print what's just happened
        }
      } // if (current_Button_State == HIGH)
    } // (reading != current_Button_State)
  } // if ((t_now - last_Debounce_Time) > debounce_Delay)

  last_Button_State = reading; // Don't forget to update the Button State (Previous Iteration) to this current reading

  // Main Loop, Part 2) Delay before Control start --------------------------------------------
  if (delay_Active) { // When delay (e.g. 5 s) before control start is active, within this 0–5 s, update control_Enabled state

  unsigned long elapsed = t_now - activation_Time; // time measuring from activation_Time (Button Pressed)

  if (elapsed >= delay_Before_Start) {  // If beyond delay interval, update states as follows:
    delay_Active = false;               // Not in delay interval
    control_Enabled = true;             // Control is enabled or active
    t_start = millis();                 // Starting counting time
    Serial.println("✅ 5s delay done → Control started"); // Print to notify delay period is finished and control will be active
  }
  else { // if within delay interval, just print to notify it's still in the delay period
    float remaining = (delay_Before_Start - elapsed) / 1000.0; // Time left before control will be active
    Serial.print("⏳ Starting in "); 
    Serial.print(remaining, 1); 
    Serial.println(" s"); // Print to show the remaining time before the control will be active
    delay(200); // delay for printing
    return;
  } // (elapsed >= delay_Before_Start)
  
  } // if (delay_Active)

  // Main Loop, Part 3) Main Control Start ------------------ 
  if (control_Enabled) {

  float t_sec = (t_now - t_start) / 1000.0;

  //read encoder count
  long count_snapshot;
  noInterrupts();
  count_snapshot = encoder_count;
  interrupts();

  float reference = Generate_Square_Waveform(t_sec, ref_Amplitude, ref_Frequency, ref_PhaseShift);
  //float reference = Generate_Sine_Waveform(t_sec, ref_Amplitude, ref_Frequency, ref_PhaseShift);
  float r = reference;

  // Y: Measurment Output
  angle = (360.0 * count_snapshot)/(CPR* 6.25 * count_Mode);
  float y = angle;

  //PID Controller
  float error = r-y;
  unsigned long dt = t_now - prev_time;
  float Ts = dt;
  if (dt == 0) dt = 1; //avoid iivision by zero 

  //Proportional Gain (Kp)
  float U_kp = kp * error;
  //Derivative Gain (Kd)
  float U_kd = kd * (error - prev_error) / (dt/1000.0);
  //Intregral Gain (Ki)
  float U_ki = ki * integral;
  //U_PID
  float U_PID = U_kp + U_ki + U_kd;

  //Contrl Saturation Limit amd Anti-Windup
  float U_Sat = U_PID;
  if (U_Sat > 255) U_Sat = 255;
  if (U_Sat < -255) U_Sat = -255;

  //Anti-Windup: update integral ONLY if output not saturated
  if (U_Sat == U_PID) {
    integral += error * (dt / 1000.0);
  }
  // Set motor direction and PWM
  int pwm_val = 30;
  int pwm_delay = 1000;

  Drive_Motor(U_Sat);

 // Serial.print("In Main Control Loop, t = "); Serial.print(t_sec, 1); Serial.println(" s");
  //Serial.print("PWM value = "); Serial.println(pwm_val);

  //Serial.print("t = "); Serial.print(t_sec,1); Serial.print(" s");
  //Serial.print(", encoderA = "); Serial.print(encoderAvalue);
  //Serial.print(", encoderB = "); Serial.print(encoderBvalue);
  //Serial.print(", Angle = "); Serial.print(angle); Serial.println(" deg");
  float neterror = fabs((reference - angle)/(reference))*100.0;
  if (t_now - lastPrintTime >= plotInterval) {
    lastPrintTime += plotInterval;
    Serial.print(t_sec, 3); Serial.print(", ");
    Serial.print(reference, 2); Serial.print(", ");
    Serial.print(angle, 2); Serial.print(", ");
    Serial.print(U_Sat, 1); Serial.print(", ");
    Serial.print(neterror); Serial.println("%"); 
  }
  prev_error = error;
  prev_time = t_now;

  delay(1);
  return;
  } // if (control_Enabled) {
    

  // ----------------- STANDBY -----------------
  if (!control_Enabled && !delay_Active) {
    static unsigned long lastPrint = 0;
    if (t_now - lastPrint >= 500) {
      Serial.println(" Standby...");
      lastPrint = t_now;
    }
  }


} // main loop() {


// Function ****************

// Function to control motor with signed PWM
//pwmVal range: -255 ... 255

void Drive_Motor(int pwmVal) {

  //pwmVal = constrain(pwmVal, -30, 30);

  if (pwmVal > 0) {
    // CW
    analogWrite(Pin_IN1, abs(pwmVal)); 
    analogWrite(Pin_IN2, 0);
  }

  else if (pwmVal < 0) {
    // CCW
    analogWrite(Pin_IN1, 0); 
    analogWrite(Pin_IN2, abs(pwmVal));
  }

  else {
    // Stop
    analogWrite(Pin_IN1, 0); 
    analogWrite(Pin_IN2, 0);
  }
}

void ISR_4X(){
  int MSB = digitalRead(encoderA);
  int LSB = digitalRead(encoderB);
  int encoded = (MSB << 1) | LSB;
  int sum = (last_Encoded << 2) | encoded;

  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000)
    encoder_count--;
  else if (sum == 0b0010 || sum == 0b0100 || sum == 0b1101 || sum == 0b1011)
    encoder_count++;

  last_Encoded = encoded;
}

//Function 02 :
float Generate_Square_Waveform(float t, float Amax, float freq, float phase_shift_deg) {

  float ref_output = 0;
  float T = 1.0 / freq;
  float phase_shift_time = (phase_shift_deg / 360.0)* T;

  float t_mod = fmod(t + phase_shift_deg, T);

  if (t_mod <= T/2.0) {
    ref_output = +Amax;
  }
  else {
    ref_output = -Amax;
  }

  return ref_output;
}

//Function 03 :
float Generate_Sine_Waveform(float t, float Amax, float freq, float phase_shift_deg) {

  float ref_output = 0;
  
  float phase_shift_rad = (phase_shift_deg / 360.0f) * 2.0f * M_PI;

  ref_output = Amax * sinf(2.0f * M_PI * freq * t + phase_shift_rad);

  return ref_output;
}



