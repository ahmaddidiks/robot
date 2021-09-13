#include <SerialCommand.h>

#define pul_pin 7
#define dir_pin 6
#define ena_pin 5

#define velDefault 720;

SerialCommand SerialCmd;

float pos = 0;    // deg
float vel = 0;    // deg/s
int ppr = 6400;
bool ena = false;
int modeState = 0;

float _velSet = 0;
long _pulseCount = 0;
long _pulseCountTarget = 0;
long _stepperLastT = 0;
bool _stepperState = LOW;

void setup() {
  //setCpuFrequencyMhz(240);

  pinMode(pul_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(ena_pin, OUTPUT);

  Serial.begin(57600);
  SerialCmd.begin(Serial);

  SerialCmd.addCommand((char*)"MODE", NULL, modeGet, modeSet, NULL);
  SerialCmd.addCommand((char*)"POS", NULL, posGet, posSet, NULL);
  SerialCmd.addCommand((char*)"POSRST", NULL, NULL, NULL, posRstExc);
  SerialCmd.addCommand((char*)"VEL", NULL, velGet, velSet, NULL);
  SerialCmd.addCommand((char*)"ENA", NULL, enaGet, enaSet, NULL);
  SerialCmd.addCommand((char*)"PPR", NULL, pprGet, pprSet, NULL);
}

void loop() {
  switch (modeState) {
    case 0:
      _velSet = vel;
      break;

    case 1:   // Position Control
      if (_pulseCount < _pulseCountTarget) _velSet = vel;
      else if (_pulseCount > _pulseCountTarget) _velSet = -vel;
      else _velSet = 0;
      break;

    default:
      break;
  }

  if ((_velSet != 0) && (micros() - _stepperLastT >= getDelayDuration(abs(_velSet)))) {
    if (_velSet < 0) {
      digitalWrite(dir_pin, HIGH);
      _pulseCount--;
    }
    else {
      digitalWrite(dir_pin, LOW);
      _pulseCount++;
    }
    digitalWrite(pul_pin, _stepperState);
    _stepperState = !_stepperState;
    _stepperLastT = micros();
  }

  SerialCmd.loop();
}

long getDelayDuration(float degPerSec) {
  if (degPerSec == 0) return 0;
  float pulsePerSec = degPerSec / 360 * ppr;
  return (0.5 / pulsePerSec * 1000000);
}

void posGet() {
  Serial.println(pos, 2);
}

void posSet() {
  char *arg = SerialCmd.next();
  if (arg != NULL) {
    pos = atof(arg);
    _pulseCountTarget = pos / 360 * ppr * 2;
    Serial.println("OK");
  }
}

void velGet() {
  Serial.println(vel, 2);
}

void velSet() {
  char *arg = SerialCmd.next();
  if (arg != NULL) {
    vel = atof(arg);
    Serial.println("OK");
  }
}

void pprGet() {
  Serial.println(ppr);
}

void pprSet() {
  char *arg = SerialCmd.next();
  if (arg != NULL) {
    ppr = atoi(arg);
    Serial.println("OK");
  }
}

void enaGet() {
  Serial.println(ena);
}

void enaSet() {
  char *arg = SerialCmd.next();
  if (arg != NULL) {
    if ((atoi(arg) == 0) || (atoi(arg) == 1)) {
      ena = atoi(arg);
      digitalWrite(ena_pin, ena);
      Serial.println("OK");
    } else {
      Serial.println("ERROR");
    }
  }
}

void modeGet() {
  Serial.println(modeState);
}

void modeSet() {
  char *arg = SerialCmd.next();
  if (arg != NULL) {
    if ((atoi(arg) >= 0) && (atoi(arg) <= 2)) {
      modeState = atoi(arg);
      if (modeState == 0) {
        vel = 0;
        _pulseCount = 0;
        _pulseCountTarget = 0;
      }
      else if (modeState == 1) {
        vel = velDefault;
        _pulseCount = 0;
        _pulseCountTarget = 0;
      }
      Serial.println("OK");
    } else {
      Serial.println("ERROR");
    }
  }
}

void posRstExc() {
  _pulseCount = 0;
  _pulseCountTarget = 0;
  pos = 0;
  Serial.println("OK");
}
