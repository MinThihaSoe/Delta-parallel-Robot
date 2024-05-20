//Trig constants
const double pi = 3.1415;        // pi
const double tan30 = 0.57735;    // tan(30 degrees)
const double cos120 = -0.5;      // cos(120 degrees)
const double sin120 = 0.866025;  // sin(120 degrees)
const float ADCtodegree = 0.26392961876;

//Delta robot parameters
const double l1 = 300.0;   // Length of active link
const double l2 = 800.0;   // Length of passive link
const short R_base = 370;  // Base radius
const short R_end = 48;    // End-effector radius

const int Stepstates = 8;
// [A1, B1, A2, B2]
int StepSequence[Stepstates][4] = {
  { 0, 1, 1, 1 },  // Step 0 [7]
  { 0, 0, 1, 1 },  // Step 1 [3]
  { 1, 0, 1, 1 },  // Step 2 [11]
  { 1, 0, 0, 1 },  // Step 3 [9]
  { 1, 1, 0, 1 },  // Step 4 [13]
  { 1, 1, 0, 0 },  // Step 5 [12]
  { 1, 1, 1, 0 },  // Step 6 [14]
  { 0, 1, 1, 0 },  // Step 7 [6]
};

volatile int iA = 0;  // Initialize steps for Motor 1
volatile int iB = 0;  // Initialize steps for Motor 2
volatile int iC = 0;  // Initialize steps for Motor 3

//Change stepper motor 1 direction and stepping mode
volatile bool state0 = 0;  // 0 = CCW, 1 = CW
volatile bool state3 = 0;  // 0 = full step, 1 = half step

//Change stepper motor 2 direction and stepping mode
volatile bool state2 = 0;  // 0 = CCW, 1 = CW
volatile bool state4 = 0;  // 0 = full step, 1 = half step

//Change stepper motor 3 direction and stepping mode
volatile bool state5 = 0;  // 0 = CCW, 1 = CW
volatile bool state6 = 0;  // 0 = full step, 1 = half step

// Setting ADC variable for conversion
const byte channel_no = 3;
volatile uint32_t ADC_val[channel_no] = { 0, 0, 0 };
volatile byte channel = 0;  // Analog channel read start at A0

// Presets for swich 1 to change trajectory direction
volatile bool state1 = 0;  // 0 = CCW, 1 = CW
volatile bool SW1;
volatile bool preSW1 = 1;  // preset = 1 to start at CW

// Presets for swich 2 to home
volatile bool Hstate = 0;  // 0 = Normal, 1 = Home
volatile bool SW2;
volatile bool preSW2 = 1;  // preset = 1 to start at Normal

// Presets for swich 3 to Emergency Break
volatile bool Estate = 0;  // 0 = ok, 1 = Emergency break
volatile bool SW3;
volatile bool preSW3 = 1;  // preset = 1 to start at ok

//Desired Waypoints
volatile uint32_t wayx[6] = { 0, 0, 0, 0, 0, 0 };
volatile uint32_t wayy[6] = { 300, 300, -300, -300, -300, 300 };
volatile uint32_t wayz[6] = { -800, -700, -700, -800, -700, -700 };
volatile long step = 0;

//Initializing confirm time
volatile long confirmtime = 0;

//Target points
volatile long Tag_x;
volatile long Tag_y;
volatile long Tag_z;

//Desired Joint angle
volatile double ThetaA;
volatile double ThetaB;
volatile double ThetaC;

// Current Joint angle
volatile float theta1;
volatile float theta2;
volatile float theta3;

//Setting PID gains:
static float kp = 0.2042041;
static float kd = 0.010006;

//Contorl output
volatile float u1;
volatile float u2;
volatile float u3;

//Errors of each joints
volatile float error1 = 0;
volatile float error1_last = 0;
volatile float error2 = 0;
volatile float error2_last = 0;
volatile float error3 = 0;
volatile float error3_last = 0;


void setup() {
  cli();  // Disables global interrupts
  Serial.begin(9600);

  // Initializing for the stepper motor 1&2, setting PA0 to PA7 to output
  DDRA = 0b11111111;
  PORTA = 0b00000000;
  // Initializing for the stepper motor 3, setting PC0 to PC3 to output
  // Also setting PC4 and PC5 to forward and reverse LED in indicitor
  DDRC = 0b00111111;
  PORTC = 0b00000000;
  // PL0 is changing pick and place direction switch
  // PL1 is Home button
  // PL2 is Emergency break switch
  DDRL = 0b00000000;  // Setting PL0 and PL2 to input pins
  PORTL = 0b0000111;  // Sinking the output (Ensuring no output at start)

  // Timer 0 Trajectory generation (8 bit timer)
  TCCR0A = 0b00000010;  // Selecting CTC mode
  TCCR0B = 0b00000101;  // clk/64 prescalar determined by Bit [2:0]
                        // Bit [4:3] = 01, Waveform Generation Mode=CTC
  OCR0A = 155;          // Sampling with 50 Hz 0.02 s
  TIMSK0 = 0b00000010;  // Bit [1] = 1, Output Compare A Match Interrupt Enable

  // Timer 1 for Stepper motor 1 speed control
  TCCR1A = 0b00000000;
  TCCR1B = 0b00001011;  // clk/64 prescalar determined by Bit [2:0]
                        // Bit [4:3] = 01, Waveform Generation Mode=CTC
  TCCR1C = 0b00000000;
  OCR1A = 2499;         // start with 100 Hz
  TIMSK1 = 0b00000010;  // Bit [1] = 1, Output Compare A Match Interrupt Enable

  // Timer 3 for Stepper motor 2 speed control
  TCCR3A = 0b00000000;
  TCCR3B = 0b00001011;  // clk/64 prescalar determined by Bit [2:0]
                        // Bit [4:3] = 01, Waveform Generation Mode=CTC
  TCCR3C = 0b00000000;
  OCR3A = 2499;         // start with 100 Hz
  TIMSK3 = 0b00000010;  // Bit [1] = 1, Output Compare A Match Interrupt Enable

  // Timer 5 for Stepper motor 2 speed control
  TCCR5A = 0b00000000;
  TCCR5B = 0b00001011;  // clk/64 prescalar determined by Bit [2:0]
                        // Bit [4:3] = 01, Waveform Generation Mode=CTC
  TCCR5C = 0b00000000;
  OCR5A = 2499;         // start with 100 Hz
  TIMSK5 = 0b00000010;  // Bit [1] = 1, Output Compare A Match Interrupt Enable

  // Timer 4 settings for controller
  TCCR4A = 0b00000000;
  TCCR4B = 0b00001011;  // clk/64 prescalar determined by bit [0:2]
  OCR4A = 499;          // OCR3A = (16,000,000/(64*500))-1 for 500 Hz (Internal clock = 16million)
  TIMSK4 = 0b00000010;  // Bit [1] = Timer 4 output compare A Match Interrupt Enable

  // Implementing ADC with Interrupt
  ADCSRA = 0b10101111;  // Bit [7] = 1, ADC Enable at register A
                        // Bit [5] = 1, Auto trigger Enable
                        // Bit [3] = 1, Interrupt Enable
                        // Bit [2:0] = 111, Prescaler = 128
  ADCSRB = 0b00000000;  // Bit [3] = MUX5 = 0 (Analog comparator)
                        // Bit [2:0] = 000, Free running mode
  ADMUX = 0b01000000;   // Bit [4:0] select analog channel and gain selection bits
                        // Bit [3:0] = 0000 = ADC0 = A0pin
  ADCSRA |= (1 << 6);   // Turn on the 6th bit and start ADC conversion

  //Initialize the FWD led :)
  PORTC = (PORTC & ~(1 << 5)) | (1 << 4);
  sei();                // Re−enables global interrupts
}

void loop() {
  // Trigger switch 1 logic to change direction
  SW1 = ((PINL & (1 << 0)) >> 0);
  if (SW1 == 1 && preSW1 == 0) {
    if (state1 == 0) {
      state1 = 1;  //CW
      PORTC = (PORTC & ~(1 << 5)) | (1 << 4);  // Turn on bit 0 and turn off bit 1
    } else {
      state1 = 0;  //CCW
      PORTC = (PORTC & ~(1 << 4)) | (1 << 5);  // Turn on bit 0 and turn off bit 1
    }
  }
  preSW1 = SW1;

  // Trigger switch 2 logic for Home
  SW2 = ((PINL & (1 << 1)) >> 1);
  if (SW2 == 1 && preSW2 == 0) {
    if (Hstate == 0) {
      Hstate = 1;
    } else {
      Hstate = 0;
    }
  }
  preSW2 = SW2;

  // Trigger switch 3 logic for Emergency
  SW3 = ((PINL & (1 << 2)) >> 2);
  if (SW3 == 1 && preSW3 == 0) {
    if (Estate == 0) {
      Estate = 1;
    } else {
      Estate = 0;
    }
  }
  preSW3 = SW3;

  theta1 = (ADC_val[0] * ADCtodegree) - 135.13;
  theta2 = (ADC_val[1] * ADCtodegree) - 135.13;
  theta3 = (ADC_val[2] * ADCtodegree) - 135.13;

  if (confirmtime >= 100) {  // confirm time is 2 secs (0.02*100)
    if (state1 == 1) {       // CW direction
      step++;
      if (step >= 6) {
        step = 0;
      }
      confirmtime = 0;
    } else {  // CCW direction
      step--;
      if (step < 0) {
        step = 5;
      }
      confirmtime = 0;
    }
  }

  //Trajectory
  if (Hstate == 0){
  Tag_x = wayx[step];
  Tag_y = wayy[step];
  Tag_z = wayz[step];
  }else{
  Tag_x = wayx[0];
  Tag_y = wayy[0];
  Tag_z = wayz[0];
  step = 0; 
  }

  double y1 = -0.5 * tan30 * R_base;        // f/2 * tan(30 deg)
  double y0 = Tag_y - 0.5 * tan30 * R_end;  // shift center to edge
  double aV = (Tag_x * Tag_x + Tag_y * Tag_y + Tag_z * Tag_z + l1 * l1 - l2 * l2 - y1 * y1) / (2.0 * Tag_z);
  double bV = (y1 - y0) / Tag_z;
  double dV = -(aV + bV * y1) * (aV + bV * y1) + l1 * (bV * bV * l1 + l1);
  if (dV < 0) {
    Serial.println("Inverse kinematics error: non-existing solution!");
  } else {
    double yj = (y1 - aV * bV - sqrt(dV)) / (bV * bV + 1);  // choosing outer solution
    double zj = aV + bV * yj;
    ThetaA = atan2(-zj, y1 - yj) * 180.0 / pi;

    double Tag_xB = Tag_x * cos120 + Tag_y * sin120;
    double Tag_yB = Tag_y * cos120 - Tag_x * sin120;
    double y0B = Tag_yB - 0.5 * tan30 * R_end;
    double aVB = (Tag_xB * Tag_xB + Tag_yB * Tag_yB + Tag_z * Tag_z + l1 * l1 - l2 * l2 - y1 * y1) / (2.0 * Tag_z);
    double bVB = (y1 - y0B) / Tag_z;
    double dVB = -(aVB + bVB * y1) * (aVB + bVB * y1) + l1 * (bVB * bVB * l1 + l1);
    if (dVB < 0) {
      Serial.println("Inverse kinematics error: non-existing solution!");
    } else {
      double yjB = (y1 - aVB * bVB - sqrt(dVB)) / (bVB * bVB + 1);
      double zjB = aVB + bVB * yjB;
      ThetaB = atan2(-zjB, y1 - yjB) * 180.0 / pi;

      double Tag_xC = Tag_x * cos120 - Tag_y * sin120;
      double Tag_yC = Tag_y * cos120 + Tag_x * sin120;
      double y0C = Tag_yC - 0.5 * tan30 * R_end;
      double aVC = (Tag_xC * Tag_xC + Tag_yC * Tag_yC + Tag_z * Tag_z + l1 * l1 - l2 * l2 - y1 * y1) / (2.0 * Tag_z);
      double bVC = (y1 - y0C) / Tag_z;
      double dVC = -(aVC + bVC * y1) * (aVC + bVC * y1) + l1 * (bVC * bVC * l1 + l1);
      if (dVC < 0) {
        Serial.println("Inverse kinematics error: non-existing solution!");
      } else {
        double yjC = (y1 - aVC * bVC - sqrt(dVC)) / (bVC * bVC + 1);
        double zjC = aVC + bVC * yjC;
        ThetaC = atan2(-zjC, y1 - yjC) * 180.0 / pi;
      }
    }
  }

  // Serial.print(Tag_x);  //A2
  // Serial.print(",");
  // Serial.print(Tag_y);  //A1
  // Serial.print(",");
  // Serial.print(Tag_z);  //A0
  // Serial.print(",");
  // Serial.print(confirmtime);
  // Serial.print(",");
  // Serial.print(state1);
  // Serial.print(",");
  // Serial.print(ThetaA);
  // Serial.print(",");
  // Serial.print(ThetaB);
  // Serial.print(",");
  // Serial.print(ThetaC);
  // Serial.print(",");
  Serial.print(ThetaA);
  Serial.print(",");
  Serial.print(theta1);
  Serial.print(",");
  Serial.print(error1);
  Serial.print(",");
  Serial.print(ThetaB);
  Serial.print(",");
  Serial.print(theta2);
  Serial.print(",");
  Serial.print(error2);
  Serial.print(",");
  Serial.print(ThetaC);
  Serial.print(",");
  Serial.print(theta3);
  Serial.print(",");
  Serial.print(error3);
  Serial.print(",");
  Serial.println();
}

ISR(ADC_vect) {
  ADC_val[channel] = ADC;  // Store the ADC reading in the ADC_val matrix
  channel++;
  if (channel > (channel_no - 1)) {
    channel = 0;
  }
  if (channel == (channel_no - 1)) {
    ADMUX = (ADMUX & 0b11100000);  //channel = 2 => switch to channel 0
  } else {
    ADMUX = (ADMUX & 0b11100000) | channel + 1;
  }
}

ISR(TIMER0_COMPA_vect) {
  if (abs(error1) < 1.5 && abs(error2) < 1.5 && abs(error3) < 1.5 ) {
    confirmtime++;
  }
}

ISR(TIMER1_COMPA_vect) {
  if (Estate == 0) {
    if (state0 == 0) {    // Forward
      if (state3 == 0) {  // Full step sequence
        iA += 2;
        // If the step is odd, correct it by going back one step
        if (iA & 0b00000001) {
          iA--;
        }
        if (iA > 6) {  // reset step i
          iA = 0;
        }
      } else {  // Half step sequence
        iA++;
        if (iA > 7) {  // reset step i
          iA = 0;
        }
      }
    } else {              // Reverse
      if (state3 == 0) {  // Full step sequence
        iA -= 2;
        if (iA & 0b00000001) {  // If the step is odd, correct it by going forward one step
          iA++;
        }
        if (iA < 0) {  // loop condition
          iA = 6;
        }
      } else {  // Half step sequence
        iA--;
        if (iA < 0) {
          iA = 7;
        }
      }
    }
    uint8_t preservedBits = PORTA & 0xF0; // Preserve bits 4 to 7
    uint8_t MOT1step = (StepSequence[iA][0] << 0) | (StepSequence[iA][1] << 1) | (StepSequence[iA][2] << 2) | (StepSequence[iA][3] << 3);
    PORTA = preservedBits | MOT1step; // Update bits 0 to 3 while preserving bits 4 to 7
  }
}

ISR(TIMER3_COMPA_vect) {
  if (Estate == 0) {
    if (state2 == 0) {    // Forward
      if (state4 == 0) {  // Full step sequence
        iB += 2;
        // If the step is odd, correct it by going back one step
        if (iB & 0b00000001) {
          iB--;
        }
        if (iB > 6) {  // reset step i
          iB = 0;
        }
      } else {  // Half step sequence
        iB++;
        if (iB > 7) {  // reset step i
          iB = 0;
        }
      }
    } else {              // Reverse
      if (state4 == 0) {  // Full step sequence
        iB -= 2;
        if (iB & 0b00000001) {  // If the step is odd, correct it by going forward one step
          iB++;
        }
        if (iB < 0) {  // loop condition
          iB = 6;
        }
      } else {  // Half step sequence
        iB--;
        if (iB < 0) {
          iB = 7;
        }
      }
    }
    uint8_t preservedBits2 = PORTA & 0x0F; // Preserve bits 0 to 3
    uint8_t MOT2step = (StepSequence[iB][0] << 4) | (StepSequence[iB][1] << 5) | (StepSequence[iB][2] << 6) | (StepSequence[iB][3] << 7);
    PORTA = preservedBits2 | MOT2step; // Update bits 4 to 7 while preserving bits 0 to 3
  }
}

ISR(TIMER5_COMPA_vect) {
  if (Estate == 0) {
    if (state5 == 0) {    // Forward
      if (state6 == 0) {  // Full step sequence
        iC += 2;
        // If the step is odd, correct it by going back one step
        if (iC & 0b00000001) {
          iC--;
        }
        if (iC > 6) {  // reset step i
          iC = 0;
        }
      } else {  // Half step sequence
        iC++;
        if (iC > 7) {  // reset step i
          iC = 0;
        }
      }
    } else {              // Reverse
      if (state6 == 0) {  // Full step sequence
        iC -= 2;
        if (iC & 0b00000001) {  // If the step is odd, correct it by going forward one step
          iC++;
        }
        if (iC < 0) {  // loop condition
          iC = 6;
        }
      } else {  // Half step sequence
        iC--;
        if (iC < 0) {
          iC = 7;
        }
      }
    }
    uint8_t preservedBits3 = PORTC & 0xF0; // Preserve bits 4 to 7
    uint8_t MOT3step = (StepSequence[iC][0] << 0) | (StepSequence[iC][1] << 1) | (StepSequence[iC][2] << 2) | (StepSequence[iC][3] << 3);
    PORTC = preservedBits3 | MOT3step; // Update bits 0 to 3 while preserving bits 4 to 7
  }
}

ISR(TIMER4_COMPA_vect) {
  error1 = ThetaA - theta1;      // ThetaA is desired, theta1 is current
  error2 = ThetaB - theta2;      // ThetaB is desired, theta2 is current
  error3 = ThetaC - theta3;      // ThetaB is desired, theta2 is current

  u1 = (kp * error1) + (kd * (error1 - error1_last) / 500);  //V_in
  if (u1 < 0) {
    u1 = -1 * u1;
    state0 = 0;
  } else {
    state0 = 1;
  }
  if (u1 > 5) {
    u1 = 5;
    state3 = 1;  // if control output needs to be greater than 5, swich to half step (increase speed)
  } else {
    state3 = 0;  // Slow down a bit
  }
  OCR1A = int((-4749.8 * u1) + 24999);  //also adjust the speed of the motor between 10 step/s to 200 step/s
  error1_last = error1;

  u2 = (kp * error2) + (kd * (error2 - error2_last) / 500);  //V_in
  if (u2 < 0) {
    u2 = -1 * u2;
    state2 = 0;
  } else {
    state2 = 1;
  }
  if (u2 > 5) {
    u2 = 5;
    state4 = 1;  // if control output needs to be greater than 5, swich to half step (increase speed)
  } else {
    state4 = 0;  // Slow down a bit
  }
  OCR3A = int((-4749.8 * u2) + 24999);  //also adjust the speed of the motor between 10 step/s to 200 step/s
  error2_last = error2;

  u3 = (kp * error3) + (kd * (error3 - error3_last) / 500);  //V_in
  if (u3 < 0) {
    u3 = -1 * u3;
    state5 = 0;
  } else {
    state5 = 1;
  }
  if (u3 > 5) {
    u3 = 5;
    state6 = 1;  // if control output needs to be greater than 5, swich to half step (increase speed)
  } else {
    state6 = 0;  // Slow down a bit
  }
  OCR5A = int((-4749.8 * u3) + 24999);  //also adjust the speed of the motor between 10 step/s to 200 step/s
  error3_last = error3;  
}