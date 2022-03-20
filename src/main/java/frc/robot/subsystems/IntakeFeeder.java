// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  private Timer pneumaticTimer;
  private Timer intakeBadTimer;

  private int id;
	private boolean doPeriodic = false;

  private boolean prevFeederSensor;

  public enum State {
    kOff, kFillToFeeder, kFillToFeederBadBall, kFillToIntake, kFillToIntakeBadBall, kShootFeeder, kShootFeederIntake, kPurgeFeederIntake, kPurgeIntake
  }

  private State state;
  private State prevState;

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

    //Feeder stuff
    feederMotor = new TalonSRX(Constants.FEEDER_TOP_MOTORS[id]);
    feederMotor.setInverted(id == 0);
    feederMotor.enableVoltageCompensation(true);
    feederMotor.configVoltageCompSaturation(12.5);
    
    feederDetector = new DigitalInput(Constants.FEEDER_BALL_DETECTORS[id]);

    //intake Stuff
    //TODO: make separate solenoids in Constants
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_FORWARD, Constants.INTAKE_SOLENOID_REVERSE);
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

    //if we are in the intake modes, check if we have a bad ball
    if( (state == State.kFillToFeeder) && !correctColor()){
      state = State.kFillToFeederBadBall;
    }else if( (state == State.kFillToIntake) && !correctColor()){
      state = State.kFillToIntakeBadBall;
      //This 
    }

    //TODO: decide if you would rather use an array for these
    double feederSpeed = 0.0;
    double intakeSpeed = 0.0;

    //TODO: put in the intake and feeder speeds in each case
    switch(state){
      case kShootFeeder:
        //shooting just one ball

        break;
      case kShootFeederIntake:
        //shooting both balls

        break;
      case kPurgeFeederIntake:
        //purging both balls
        
        break;
      case kPurgeIntake:
        //purging intake ball

        break;
      case kFillToIntakeBadBall:
        //pushing out ball as it the wrong color

        break;
      case kFillToFeederBadBall:

        //TODO: run a debounce on this logic too, see end of periodic
        if(!getFeederDetector()){
          //drop through
          state = State.kFillToFeeder;
        }
        else{
          break;
        }
      case kFillToFeeder:
        //filling the robot until the Feeder sensor is detected
        if(getFeederDetector()){
          //if we see a ball change state and fall to next case
          state = State.kFillToIntake;
        }
        else{
          break;
        }
      case kFillToIntake:
        //filling the robot until the intake sensor is detected
        
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
        break;
      default:
        feederSpeed = 0.0;
        intakeSpeed = 0.0;
        System.out.println("default ball handler case reached");
    }

    //if we change state, check to see if we need to change the pneumatics etc
    if(state != prevState){
      //TODO: write pneumatic stuff
    }

    intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    //TODO:as above, set the feeder speed once at the end, this keeps the motor from rapid changes as the case will change motor speeds multiple times, sometimes

    //used in the debounce of the feeder ball sensor
    prevFeederSensor = getFeederDetector();
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
    //TODO: write color sensor code
    return true;
  }
}
