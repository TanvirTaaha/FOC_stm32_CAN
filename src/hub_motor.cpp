/**
 *  Hall sensor example code
 *
 * This is a code intended to test the hall sensors connections and to demonstrate the hall sensor setup.
 *
 */

/**
 *  Hall sensor example code
 *
 * This is a code intended to test the hall sensors connections and to demonstrate the hall sensor setup.
 *
 */

// Open loop motor control example

#include "motor.h"

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(15, 0.4, 18);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, 15);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

void setup()
{
  setup_can_comm();
  Serial.begin(115200);
  // driver config
  Serial.println("started");
  // currentSense.gain_a *= -1;
  // currentSense.gain_b *= -1;
  // currentSense.gain_c *= -1;
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);
  Serial.println("Motor ready! 444");
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  // driver.voltage_limit = 6;
  driver.init();
  currentSense.linkDriver(&driver);

  Serial.println("2Motor ready!");
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.linkCurrentSense(&currentSense);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // currnet = resistance * voltage, so try to be well under 1Amp
  motor.voltage_sensor_align = 3;
  motor.voltage_limit = 1; // [V]
  motor.velocity_limit = 10;
  // motor.voltage_sensor_align = 1;
  motor.current_limit = 2;

  // open loop control config
  // motor.phase_resistance = 0.8;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity;

  motor.PID_velocity.P = .8;
  motor.PID_velocity.I = 9;
  motor.PID_velocity.D = 0.005;
  motor.LPF_velocity.Tf = 0.01;
  // default voltage_power_supply

  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL;
  motor.monitor_downsample = 200;
  // init motor hardware

  motor.init();
  currentSense.init();
  motor.linkCurrentSense(&currentSense);

  motor.zero_electric_angle = 2.09; // rad
  motor.sensor_direction = Direction ::CCW;

  // currentSense.skip_align = true;
  // currentSense.driverAlign(motor.voltage_sensor_align);

  motor.initFOC();
  Serial.println("Motor ready! 1");
}

void loop()
{
  static int i = 0;
  if (i == 5000)
  {
#ifdef DEBUG
    Serial.printf("dc_current:%f\n", currentSense.getDCCurrent());
    Serial.printf("target_velocity:%f\n", target_velocity);
    Serial.printf("speed:%f, position:%f\n", sensor.getVelocity(), sensor.getAngle());
#endif
    loop_can_comm();
    i = 0;
  }
  i++;
  // serialLoop();
  // motor.monitor();
  setTxSpeedAndPos(sensor.getVelocity(), sensor.getAngle());
  motor.loopFOC();
  motor.move(target_velocity);
}
