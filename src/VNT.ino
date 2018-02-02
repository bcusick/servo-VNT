
/*
   based on "Standalone VNT Controller" by DMN - http://dmn.kuulalaakeri.org/vnt-lda/
  - PID loop update
  - added support to control servo actuated turbo.  i.e. Garrett GTB2056V
  - condensed code to core functionality
  -TODO  add support to select from multiple maps

*/


#include <ResponsiveAnalogRead.h>
#include <PID_v1.h>
#include <servoCont.h>


#define PIN_BUTTON A5
#define PIN_HEARTBEAT 13

#define PIN_MAP A1
#define PIN_TPS A0
#define PIN_EMP A2

#define PIN_RPM_TRIGGER 2
#define PIN_VNT_N75 11

ResponsiveAnalogRead map_read(PIN_MAP, false);  // no sleep on the MAP read; need the resolution
ResponsiveAnalogRead tps_read(PIN_TPS, true);
ResponsiveAnalogRead rpm_read(0, true);

#define IDLE_MAX_RPM 1150
#define MIN_BOOST_SPOOLED 10 // kPa
#define PID_CUT_IN 1520 // rpm
#define TPS_CUT_IN 18 // ~ 7%


/* Scaling factor for your sensors - 255 divided by this should equal the full scale deflection of your sensor */
#define MAP_SCALING_KPA 0.977
#define EMP_SCALING_KPA 1.953

/* Change this if you need to adjust the scaling of the PID outputs - ie if you need finer control at smaller fractional numbers increase this
  or if you need to have large multipliers then decrease this */
#define PIDControlRatio 300

/* The resolution we use to calculate RPM - we are only going to calculate RPM ever 'n' number of teeth that pass by; otherwise we are going to have
  a jittery value.  Divide this value by the 'Teeth per Rotation' setting to know how many revolutions before we caculate RPM. */
#define rpmResolution 30

// Set loop delay times
#define SERIAL_DELAY 107 // ms
#define EXEC_DELAY 50 //ms
#define DISPLAY_DELAY 250 // ms
#define MAP_DELAY 10 //ms


// Calculate Average values
#define AVG_MAX 15

#define MAP_AXIS_TPS 0xDE
#define MAP_AXIS_RPM 0xAD
#define MAP_AXIS_KPA 0xDD
#define MAP_AXIS_CELSIUS 0xAA
#define MAP_AXIS_VOLTAGE 0xAB
#define MAP_AXIS_DUTY_CYCLE 0xAC
#define MAP_AXIS_RAW 0x0
#define MAP_AXIS_EGT 0xAE

/*
  MAP format:
  'M','2','D'   // D - interpolated maps, d - nearest neighbor
  xsize,ysize,x-axis-type,y-axis-type,output-type,
  data[xsize,ysize],
  lastX,lastY,lastRet // automatically filled when used mapLookup
*/

unsigned char auxMap[] = {
  'M', '2', 'D',
  0x6, 0x8, MAP_AXIS_RPM, MAP_AXIS_EGT, MAP_AXIS_DUTY_CYCLE, // 01 - new version
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  60, 60, 60, 60, 210, 210,
  210, 210, 210, 210, 210, 210,
  00, 00, 00,                // lastX,lastY,lastRet
};


unsigned char boostRequest[] = {
  'M', '2', 'D',
  0xC, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_KPA, // 01 - new version
  0, 0, 0, 15,  34, 44, 46, 46, 46, 43, 39, 37,
  0, 5, 15, 27, 38, 49, 51, 51, 51, 48, 43, 41,
  0, 5, 20, 32, 42, 54, 57, 57, 57, 53, 48, 45,
  0, 5, 20, 40, 55, 60, 63, 63, 63, 59, 53, 50,
  0, 5, 22, 43, 60, 67, 70, 70, 70, 65, 59, 55,
  0, 5, 24, 47, 65, 74, 78, 78, 78, 72, 66, 61,
  0, 6, 27, 58, 75, 92, 98, 98, 98, 90, 83, 76,
  0, 7, 30, 70, 92, 110, 123, 123, 123, 113, 104, 95,
  0, 9, 33, 75, 112, 138, 154, 154, 154, 141, 130, 119,
  0, 11, 41, 87, 128, 173, 193, 193, 193, 176, 162, 149,
  0, 12, 45, 85, 145, 192, 214, 214, 214, 195, 180, 166,
  00, 00, 00,                // lastX,lastY,lastRet
};

unsigned char boostDCMax[] = {
  'M', '2', 'D',
  0x8, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_DUTY_CYCLE,
  0, 204, 204, 180, 155, 140, 120, 70,
  0, 204, 204, 180, 155, 140, 120, 70,
  0, 204, 204, 175, 155, 140, 120, 70,
  0, 204, 190, 160, 135, 130, 120, 70,
  0, 204, 185, 160, 135, 130, 120, 70,
  0, 204, 180, 160, 135, 130, 120, 70,
  0, 204, 180, 155, 135, 130, 120, 70,
  0, 204, 175, 150, 135, 130, 120, 70,
  0, 204, 175, 145, 135, 125, 120, 70,
  0, 204, 180, 145, 135, 125, 120, 70,
  0, 204, 185, 145, 135, 125, 120, 70,
  00, 00, 00,                // lastX,lastY,lastRet
};

unsigned char boostDCMin[] = {
  'M', '2', 'D',
  0x9, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_DUTY_CYCLE,
  0, 50, 50, 50, 50, 50, 50, 50, 50,
  0, 121, 110, 100, 91, 83, 75, 68, 50,
  0, 121, 110, 100, 91, 83, 75, 68, 50,
  0, 121, 110, 100, 91, 83, 75, 68, 50,
  0, 110, 100, 91, 83, 75, 68, 62, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  00, 00, 00,                // lastX,lastY,lastRet
};

unsigned char n75precontrolMap[] = {
  'M', '2', 'D',
  0xC, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_DUTY_CYCLE,
  0, 90, 90, 110, 110, 100, 90, 75, 62, 50, 50, 50,
  0, 204, 204, 204, 182, 163, 145, 130, 122, 115, 106, 70,
  0, 204, 204, 198, 165, 148, 137, 126, 118, 112, 102, 70,
  0, 204, 204, 176, 160, 142, 130, 117, 107, 102, 95, 70,
  0, 204, 204, 145, 124, 115, 109, 101, 94, 87, 82, 70,
  0, 204, 204, 145, 115, 112, 104, 94, 90, 85, 78, 70,
  0, 204, 204, 132, 111, 109, 103, 94, 87, 80, 76, 70,
  0, 204, 204, 132, 110, 108, 102, 94, 86, 80, 76, 70,
  0, 204, 204, 138, 107, 103, 98, 91, 85, 80, 76, 70,
  0, 204, 204, 142, 128, 119, 110, 103, 97, 89, 82, 70,
  0, 204, 204, 160, 150, 140, 132, 121, 110, 98, 86, 70,
  00, 00, 00,              // lastX,lastY,lastRet
};



#define OPTIONS_VANESOPENIDLE 1
#define OPTIONS_VNTOUTPUTINVERTED 2

// contains configurable data. Can be stored in eeprom
struct settingsStruct {
  int tpsMin;
  int tpsMax;
  int mapMin;
  int mapMax;
  int empMin;
  int empMax;
  int rpmMax;
  int rpmTeethsPerRotation;
  unsigned char mode;
  char options;
  int boostKp;
  int boostKi;
  int boostKd;
};

settingsStruct settings;

//  contains calculated output data. calculated each run of mainloop
struct controlsStruct {
  // inputs
  volatile int    tpsInput;
  unsigned char   tpsCorrected;
  volatile int    mapInput;
  double          mapCorrected;
  volatile int    egtInput;
  unsigned char   egtCorrected;
  volatile int    empInput;
  unsigned char   empCorrected;
  char            mode; // operating mode

  // outputs

  double          vntTargetPressure;
  unsigned char   vntPositionRemapped;
  unsigned char   vntPositionDC;
  int             vntMinDc;
  int             vntMaxDc;
  int             n75precontrol;

  // calculated value
  volatile int            rpmActual;
  volatile unsigned char  rpmCorrected;
  unsigned char           statusBits;

  bool            idling;
  int             temp1;

  unsigned char   auxOutput;

  float           boostCalculatedP;
  float           boostCalculatedI;
  float           boostCalculatedD;

  double          pidOutput;

  unsigned long   lastTime;
  float           lastInput;
};

controlsStruct controls;


double Kp;
double Ki;
double Kd;

// set up VNT PID control
PID vntPid(&controls.mapCorrected, &controls.pidOutput, &controls.vntTargetPressure, Kp, Ki, Kd, P_ON_E,DIRECT);

struct avgStruct {
  unsigned char         pos;
  unsigned char         size;
  volatile unsigned int avgData[AVG_MAX];
};

avgStruct mapAvg;

char buffer[100]; // general purpose buffer, mainly used for string storage when printing from flash
unsigned long lastPacketTime;

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 64:
        mode = 0x03;
        break;
      case 256:
        mode = 0x04;
        break;
      case 1024:
        mode = 0x05;
        break;
      default:
        return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 32:
        mode = 0x03;
        break;
      case 64:
        mode = 0x04;
        break;
      case 128:
        mode = 0x05;
        break;
      case 256:
        mode = 0x06;
        break;
      case 1024:
        mode = 0x07;
        break;
      default:
        return;
    }
    // TCCR2A = 0xA3;
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


volatile unsigned int teethNo = 0;

void rpmTrigger() {
  // increase the tooth count whenever we see a tooth go by
  teethNo++;
}

unsigned long rpmMicros = 0;
unsigned long teethSeconds = 0;

void calcRpm() {
  if (teethNo > rpmResolution)
  {
    int rpm;

    detachInterrupt(0); // don't trigger increments while we're calculating

    teethSeconds = 60000000 / settings.rpmTeethsPerRotation;

    // teethSeconds is one second in microseconds / number of teeth per revolution - avoid overflowing by pre-dividing a second by the number of teeth
    rpm = (teethSeconds * teethNo) / (micros() - rpmMicros);

    // Set time to now, reset tooth count to zero to start incrementing again
    rpmMicros = micros();
    teethNo = 0;

    attachInterrupt(0, rpmTrigger, FALLING); // back to the daily grind

    // controls.rpmActual = (rpmSmoothing * rpm) + ((1.0-rpmSmoothing)*controls.rpmActual);

    rpm_read.update(rpm / 10);
    controls.rpmActual = rpm_read.getValue() * 10;

    if (controls.rpmActual > settings.rpmMax) {
      controls.rpmActual = 0;
    }
  }
}



void calcKp() {
  Kp = (float)(settings.boostKp) / PIDControlRatio;
}

void calcKi() {
  Ki = (float)(settings.boostKi) / PIDControlRatio;
}

void calcKd() {
  Kd = (float)(settings.boostKd) / PIDControlRatio;
}

void setup() {
  delay(1500);    // Wait for LCD to actually start up
  setup_lcd();

  Serial.begin(115200);
  Serial.print(F("Boot:"));

  pinMode(PIN_HEARTBEAT, OUTPUT); // DEBUG led

  pinMode(PIN_BUTTON, INPUT); // Reset switch
  digitalWrite(PIN_BUTTON, HIGH);  // activate pull up resistor

  pinMode(PIN_RPM_TRIGGER, INPUT); // Reset switch
  digitalWrite(PIN_RPM_TRIGGER, HIGH); // pullup for honeywell

  attachInterrupt(0, rpmTrigger, FALLING); // or rising!

  setPwmFrequency(PIN_VNT_N75, 128); // was 1024
  setPwmFrequency(PIN_AUX_N75, 128); // was 1024

  pinMode(PIN_VNT_N75, OUTPUT);
  pinMode(PIN_AUX_N75, OUTPUT);

  pinMode(PIN_TPS, INPUT);
  pinMode(PIN_MAP, INPUT);

  digitalWrite(PIN_TPS, LOW); // safety unconnected TPS
  digitalWrite(PIN_MAP, HIGH); // safety unconnected MAP



  mapAvg.size = AVG_MAX;

  //initial setup of kp/ki/kd
  calcKp();
  calcKi();
  calcKd();

  digitalWrite(PIN_HEARTBEAT, LOW);

  // set up screen
  layoutLCD();

  pageAbout(1); // force output
}

void loadDefaults() {
  memset(&settings, 0, sizeof(settingsStruct));
  settings.tpsMin = 85;
  settings.tpsMax = 970;
  settings.mapMin = 55;
  settings.mapMax = 975;
  settings.empMax = 1023;
  settings.egtMax = 970;
  settings.egtMin = 0;
  settings.rpmTeethsPerRotation = 4;
  settings.rpmMax = 6000;
  settings.options = 0;
  settings.boostKp = 220;
  settings.boostKi = 7;
  settings.boostKd = 15;
}


unsigned char mapValues(int raw, int mapMin, int mapMax) {
  if (raw < mapMin)
    return 0;
  if (raw >= mapMax)
    return 0xff;

  return map(raw, mapMin, mapMax, 0, 255);
}

unsigned char mapValuesSqueeze(int raw, int mapMin, int mapMax) {
  return map(raw, 0, 255, mapMin, mapMax);
}

unsigned char mapInterpolate(unsigned char p1, unsigned char p2, unsigned char pos) {
  return (p1 * (100 - pos) + p2 * pos) / 100;
}

unsigned char mapLookUp(unsigned char *mapData, unsigned char x, unsigned char y) {
  unsigned char isInterpolated = *(mapData + 2);
  unsigned char tableSizeX = *(mapData + 3);
  unsigned char tableSizeY = *(mapData + 4);
  unsigned char yPos;
  *(mapData + 8 + tableSizeX * tableSizeY) = x;
  *(mapData + 8 + tableSizeX * tableSizeY + 1) = y;

  if (tableSizeY) {
    yPos = y / (256 / (tableSizeY - 1));
  }
  else {
    yPos = 0;
  }
  unsigned char xPos = (x / (256 / (tableSizeX - 1)));
  int ofs = 8; // skip headers

  unsigned char p1 = *(mapData + ofs + (yPos * tableSizeX) + xPos);
  unsigned char p2 = *(mapData + ofs + (yPos * tableSizeX) + (((xPos + 1) >= tableSizeX) ? xPos : xPos + 1));
  unsigned char p3 = *(mapData + ofs + ((((yPos + 1) >= tableSizeY) ? yPos : yPos + 1) * tableSizeX) + xPos);
  unsigned char p4 = *(mapData + ofs + ((((yPos + 1) >= tableSizeY) ? yPos : yPos + 1) * tableSizeX) + (((xPos + 1) >= tableSizeX) ? xPos : xPos + 1));

  unsigned char ret;
  if (isInterpolated == 'D') {
    int amountX = (x % (256 / (tableSizeX - 1))) * (10000 / (256 / (tableSizeX - 1)));
    if (tableSizeY) {
      // 2D
      int amountY = (y % (256 / (tableSizeY - 1))) * (10000 / (256 / (tableSizeY - 1)));
      char y1 = mapInterpolate(p1, p2, amountX / 100);
      char y2 = mapInterpolate(p3, p4, amountX / 100);
      ret = mapInterpolate(y1, y2, amountY / 100);
    }
    else {
      // 1D
      ret = mapInterpolate(p1, p2, amountX / 100);
    }
  }
  else {
    ret = p1;
  }
  *(mapData + 8 + tableSizeX * tableSizeY + 2) = ret;
  return ret;
}


void readValuesTps() {
  tps_read.update();
  controls.tpsInput = tps_read.getValue();
}

void readValuesMap() {
  map_read.update();
  controls.mapInput = map_read.getValue();
}

void determineIdle() {
  if ( controls.tpsCorrected > 0 ) {
    controls.idling = false;
  }
  else if ( controls.rpmActual < IDLE_MAX_RPM ) {    // Accelerator is at zero and we are in the idle speed band
    controls.idling = true;
  }
  else {
    controls.idling = false;                         // Most likely coasting right now; continue proportional behavior
  }
}


////////////////////////////////////////
VNT control
/////////////////////////////////

void controlVNT() {

  double minControl;
  double maxControl;

  double toControlVNT;

  controls.rpmCorrected = mapValues(controls.rpmActual, 0, settings.rpmMax);
  controls.mapCorrected = mapValues(controls.mapInput, settings.mapMin, settings.mapMax);
  controls.tpsCorrected = mapValues(controls.tpsInput, settings.tpsMin, settings.tpsMax);

  controls.vntMaxDc = mapLookUp(boostDCMax, controls.rpmCorrected, controls.tpsCorrected);
  controls.vntMinDc = mapLookUp(boostDCMin, controls.rpmCorrected, controls.tpsCorrected);

  controls.n75precontrol = mapLookUp(n75precontrolMap, controls.rpmCorrected, controls.tpsCorrected);

  /* Look up the requested boost */
  controls.vntTargetPressure = mapLookUp(boostRequest, controls.rpmCorrected, controls.tpsCorrected);

  /* This is the available span of our DC - we can only go between min and max */
  minControl = controls.vntMinDc - controls.n75precontrol;  // this will be a negative number
  maxControl = controls.vntMaxDc - controls.n75precontrol;  // this will be a positive number

  if ( minControl > 0 ) {
    // Our MinDC map is higher than our precontrol map; oops
    minControl = 0;
  }

  if ( maxControl < 0 ) {
    // Our MaxDC map is lower than our precontrol map; oops
    maxControl = 0;
  }

  vntPid.SetOutputLimits(minControl, maxControl);

  if ((controls.idling)) {
    // If we are at idle then we don't want any boost regardless of map

    controls.vntTargetPressure = 0;                    // Display zero target pressure on the LCD at idle
    controls.mode = 0;                                 // System status = idling
    controls.pidOutput = 0;

    vntPid.SetMode(MANUAL);                            // Disable PID controller at idle

    if (settings.options & OPTIONS_VANESOPENIDLE) {
      toControlVNT = minControl;
    } else {
      toControlVNT = maxControl;
    }

  }
  else if (controls.mapCorrected <= MIN_BOOST_SPOOLED || controls.rpmActual < PID_CUT_IN || controls.tpsCorrected < TPS_CUT_IN ) {
    // If the turbo hasn't spooled up yet we're going to end up winding up the control loop; the precontrol map
    // should be more than sufficient to get things spinning

    controls.mode = 1;                                // We haven't spooled, don't integrate yet

    vntPid.SetMode(AUTOMATIC);
    vntPid.SetTunings(Kp, 0.0, Kd);

  }
  else {

    vntPid.SetMode(AUTOMATIC);
    vntPid.SetTunings(Kp, Ki, Kd);

    vntPid.Compute();

    if ( controls.pidOutput == minControl ) {
      // We are at minimum
      controls.mode = 4;
    } else if ( controls.pidOutput == maxControl ) {
      // We are at maximum
      controls.mode = 3;
    } else {
      // Normal in-range running
      controls.mode = 2;
    }

  }

  toControlVNT = round(controls.pidOutput) + controls.n75precontrol;

  controls.vntPositionDC = toControlVNT;

  /* This loop should never ever be true - a 100% output should be diff between min and max + min which should equal max
    but I'm not quite ready to remove this */
  if (controls.vntPositionDC > controls.vntMaxDc)
    controls.vntPositionDC = controls.vntMaxDc;

  /* Display these as real numbers - will make the logs more useful as we can try different values */
  controls.boostCalculatedP = vntPid.GetKp();
  controls.boostCalculatedI = vntPid.GetKi();
  controls.boostCalculatedD = vntPid.GetKd();

  unsigned char finalPos;
  finalPos = controls.vntPositionDC;

  if (settings.options & OPTIONS_VNTOUTPUTINVERTED) {
    controls.vntPositionRemapped = 255 - finalPos;
  }
  else {
    controls.vntPositionRemapped = finalPos;
  }

  // Include the time we spent processing
  controls.lastTime = millis();
}



void updateOutputValues() {
  // PWM output pins
  analogWrite(PIN_VNT_N75, controls.vntPositionRemapped);
  analogWrite(PIN_AUX_N75, controls.auxOutput);
}

bool freezeModeEnabled = false;

unsigned long serialLoop = 0;
unsigned long execLoop = 0;
unsigned long displayLoop = 0;
unsigned long mapLoop = 0;


void loop() {

  if ((millis() - mapLoop) >= MAP_DELAY) {
    readValuesMap();  // Read every loop; we're calculating an average to clean up noise.
    mapLoop = millis();
  }

  /* Actual execution will happen every EXEC_DELAY - this is where we do our actual calculations */
  if ((millis() - execLoop) >= EXEC_DELAY) {


    execTimeRead = millis();
    readValuesTps();
    readValuesEgt();
    execTimeRead = millis() - execTimeRead;


    execTimeAct = millis();

    // update output values according to input
    calcRpm();
    determineIdle();
    controlVNT();
    updateOutputValues();

    execLoop = millis();
    execTimeAct = execLoop - execTimeAct;
  }


  /* we are only going to actualy process every SERIAL_LOOP_DELAY milliseconds though we will read from our sensors every loop
    This way we can get high resolution readings from the sensors without waiting for the actual calculations to occur every
    single time

  else if ((millis() - serialLoop) >= SERIAL_DELAY) {

    unsigned char data = 0;

    // User interface for configuration and monitoring
    if (Serial.available()) {
      data = Serial.read();
      if (data >= '0' && data <= '9') {
        page = data - '0';
      }
      else if (data == ':') {
        freezeModeEnabled = !freezeModeEnabled;
      }
      else if (data == '#') {
        page = 10;
      }
      else if (data == 27) {
        data = Serial.read();
        if (data == '[') {
          data = Serial.read();
          switch (data) {
            case 'A':
              data = 'k';
              break;
            case 'B':
              data = 'j';
              break;
            case 'C':
              data = 'l';
              break;
            case 'D':
              data = 'h';
              break;

            default:
              data = 0;
          }
        }
      }
    }
    displayPage(page, data);
    serialLoop = millis();
  }

  /* The LCD also takes a while to update; only do the update every DISPLAY_DELAY ms as we don't need a blur where numbers should be */

  /*

  else if ((millis() - displayLoop) >= DISPLAY_DELAY) {
    execTimeLcd = millis();
    // We will only update the LCD every DISPLAY_DELAY milliseconds
    updateLCD();
    displayLoop = millis();
    execTimeLcd = displayLoop - execTimeLcd;
  }
}
*/
