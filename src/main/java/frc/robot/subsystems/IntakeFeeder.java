// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeFeeder extends SubsystemBase {
  private DoubleSolenoid intakeSolenoid;
	private TalonSRX intakeMotor;
	private DigitalInput intakeSensor;
  private TalonSRX feederMotor;
  private DigitalInput feederDetector;
  private ColorSensorV3 colorSensor;
  private Timer pneumaticTimer;
  private Timer intakeBadTimer;

  private int id;
	private boolean doPeriodic = false;
  private boolean pneumaticOverride;
  private boolean isRedCorrect;
  private boolean intakeUp = true;

  public enum State {
    kOff,
    kFillToFeeder,
    kFillToFeederBadBall,
    kFillToIntake,
    kFillToIntakeBadBall,
    kShootFeeder,
    kShootFeederIntake,
    kPurgeFeederIntake,
    kPurgeFeeder,
    kPurgeIntake
  }

  private State state = State.kOff;
  private State prevState = State.kOff;

  /**
	 * Do not use
	 */
	@Deprecated
  public IntakeFeeder() {
    this.doPeriodic = false;
  }

  public IntakeFeeder(int id) {
    this.id = id;
    this.doPeriodic = true;

    //TODO: move the following down  either periodic or correctColor, this is not a call, this is local info there is no time delay, this method will only be right if the robot boots with the right alliance, which can't be promised.
    this.isRedCorrect = DriverStation.getAlliance() == DriverStation.Alliance.Red;

    colorSensor = new ColorSensorV3(Constants.I2C_PORTS[id]);

    //Feeder stuff
    feederMotor = new TalonSRX(Constants.FEEDER_TOP_MOTORS[id]);
    feederMotor.setInverted(id == 0);
    feederMotor.enableVoltageCompensation(true);
    feederMotor.configVoltageCompSaturation(12.5);
    
    feederDetector = new DigitalInput(Constants.FEEDER_BALL_DETECTORS[id]);

    //intake Stuff
    //TODO: make separate solenoids in Constants
    if (id == 0) {
      intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_REVERSE);
    }
    pneumaticTimer = new Timer();

    intakeMotor = new TalonSRX(Constants.INTAKE_MOTORS[id]);
		intakeMotor.setInverted(id == 0);
		intakeMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor.enableVoltageCompensation(true);
		intakeMotor.configVoltageCompSaturation(12.5);

    intakeSensor = new DigitalInput(Constants.INTAKE_BALL_DETECTORS[id]);

    intakeBadTimer = new Timer();
  }

  @Override
  public void periodic() {
    //if empty constructor version, return
    if (!doPeriodic) {
      return;
    }

    //send sensors to DS
    SmartDashboard.putBoolean("Intake Sensor " + (id == 0 ? "Left" : "Right"), getIntakeSensor());
    SmartDashboard.putBoolean("Feeder Sensor " + (id == 0 ? "Left" : "Right"), getFeederDetector());

    //TODO: decide if you would rather use an array for these
    double feederSpeed = 0.0;
    double intakeSpeed = 0.0;
    intakeUp = true;

    switch(state){
      case kShootFeeder:
        //shooting just one ball
        feederSpeed = Constants.FEEDER_SHOOT_SPEED;
        intakeSpeed = 0;
        break;
      case kShootFeederIntake:
        //shooting both balls
        feederSpeed = Constants.FEEDER_SHOOT_SPEED;
        intakeSpeed = Constants.INTAKE_INTAKE_SPEED;
        break;
      case kPurgeFeederIntake:
        //purging both balls
        feederSpeed = Constants.FEEDER_PURGE_SPEED;
        intakeSpeed = Constants.INTAKE_PURGE_SPEED;
        break;
      case kFillToIntakeBadBall:
        feederSpeed = 0;
        intakeSpeed = Constants.INTAKE_INTAKE_SPEED;
        if (getIntakeSensor()) {
          intakeBadTimer.reset();
          intakeBadTimer.start();
          state = State.kPurgeIntake;
        }else{
          intakeUp = false;
          break;
        }
      case kPurgeIntake:
        feederSpeed = 0;
        intakeSpeed = Constants.INTAKE_PURGE_SPEED;
        intakeUp = false;
        if (intakeBadTimer.hasElapsed(0.75)) {
          state = State.kFillToIntake;
        }
        break;
      case kFillToFeederBadBall:
        feederSpeed = Constants.FEEDER_INTAKE_SPEED;
        intakeSpeed = Constants.INTAKE_INTAKE_SPEED;
        intakeUp = false;
        if (getFeederDetector()) {
          state = State.kPurgeFeeder;
        }else{
          break;
        }
      case kPurgeFeeder:
          // Assumes the shooter is running at its low speed
          feederSpeed = Constants.FEEDER_INTAKE_SPEED;
          intakeSpeed = 0;//TODO: why are we stoppping this????????? don't
          intakeUp = false;
          if (!getFeederDetector()) {
            state = State.kFillToFeeder;
          }else{
            break;
          }
      case kFillToFeeder:
        //filling the robot until the Feeder sensor is detected
        feederSpeed = Constants.FEEDER_INTAKE_SPEED;
        intakeSpeed = Constants.INTAKE_SLOW_SPEED;
        intakeUp = false;
        if (!correctColor()) {
          state = State.kFillToFeederBadBall;
          break;
        }
        if(getFeederDetector()){
          //if we see a ball change state and fall to next case
          state = State.kFillToIntake;
        }
        else{
          break;
        }
      case kFillToIntake:
        //filling the robot until the intake sensor is detected
        feederSpeed = 0;
        intakeSpeed = Constants.INTAKE_SLOW_SPEED;
        intakeUp = false;
        if(!correctColor()) {
          state = State.kFillToIntakeBadBall;
          break;
        }
        if(getIntakeSensor()){
          //if we see a ball change state and fall to next case
          state = State.kOff;
        }
        else{
          break;
        }
      case kOff:
        feederSpeed = 0.0;
        intakeSpeed = 0.0;
        intakeUp = true;
        break;
      default:
        feederSpeed = 0.0;
        intakeSpeed = 0.0;
        intakeUp = true;
        System.out.println("default ball handler case reached");
    }

    //TODO: remove previous intakeUp logic, go based on shift in State by checking change in state(use prevState) we need independant control of pneumatic arm, so this periodic isn't the only control of the intake pneumatic
    if(intakeUp) {
      pneumaticTimer.start();
    } else {
      pneumaticTimer.reset();
      pneumaticTimer.stop();
    }
  
    if (!pneumaticOverride) {
      setPneumaticDown(!pneumaticTimer.hasElapsed(0.25));
    }

    //set the motors to the speeds defined in the case structure
    intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    feederMotor.set(ControlMode.PercentOutput, feederSpeed);
    prevState = state;
  }

  /**
   * Set the state of the IntakeFeeder. The IntakeFeeder 
   * is managed by the periodic function in the 
   * IntakeFeeder. This is used to change the state of 
   * the IntakeFeeder
   * 
   * @param state uses enum State(IntakeFeeder.State)
   */
  public void setState(State state){
    this.state = state;
  }

  /**
   * returns the current state of the IntakeFeeder.
   * 
   * @return enum State (IntakeFeeder.State)
   */
  public State getState(){
    return state;
  }

  /**
   * set the pneumatic down, true to make it go down, false to bring it up
   * @param pneumaticDown
   */
  public void setPneumaticDown(boolean pneumaticDown){
    if (id == 1) {
      return;
    }
    if(pneumaticDown){
      intakeSolenoid.set(Value.kForward);
    }else{
      intakeSolenoid.set(Value.kReverse);
    }
  }

  public boolean getIntakeSensor() {
		return !intakeSensor.get();
	}

  public boolean getFeederDetector() {
    return !feederDetector.get();
  }

  public boolean correctColor(){

    //TODO: check that the sensor isConnected, polling the sensor when not connected can halt code and freeze robot, return true if not connected

    int red = colorSensor.getRed();
    int blue = colorSensor.getBlue();
    int proximity = colorSensor.getProximity();

    if (red == 0 && blue == 0 && proximity == 0) {
      // Color sensor is disconnected, attempt to reconnect
      colorSensor = new ColorSensorV3(Constants.I2C_PORTS[id]);
      return true;
    }

    int diff = red - blue;
    if (Math.abs(diff) < 100) {
      // Red / blue signal is not strong enough to make a judgment
      return true;
    }
    return (diff < 0) ^ isRedCorrect;
  }
}
