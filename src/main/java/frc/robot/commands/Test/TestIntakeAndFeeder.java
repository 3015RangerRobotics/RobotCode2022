// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.IntakeFeeder.State;

public class TestIntakeAndFeeder extends CommandBase {

  NetworkTable testTable = NetworkTableInstance.getDefault().getTable("test");

  IntakeFeeder intakeFeederLeft;
  IntakeFeeder intakeFeederRight;
  Timer timer;

  int stage = 0;

  /** Creates a new TestIntake. */
  public TestIntakeAndFeeder() {
    stage = 0;
    intakeFeederLeft = RobotContainer.intakeFeeder[0];
    intakeFeederRight = RobotContainer.intakeFeeder[1];
    addRequirements(intakeFeederLeft, intakeFeederRight);
    

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeFeederLeft.setState(State.kManual);
    intakeFeederRight.setState(State.kManual);
    timer.reset();
    timer.start();
    testTable.getEntry("Left Feeder Sensor").setBoolean(false);
    testTable.getEntry("Left Intake Sensor").setBoolean(false);
    testTable.getEntry("Right Feeder Sensor").setBoolean(false);
    testTable.getEntry("Right Intake Sensor").setBoolean(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testTable.getEntry("test stage").setNumber(stage);
    switch (stage) {
      case 0: /* runs left intake and feeder */
        intakeFeederLeft.setIntakeMotor(Constants.INTAKE_TEST_SPEED);
        intakeFeederLeft.setFeederMotor(Constants.FEEDER_TEST_SPEED);
        stage++;
        break;
      case 1: /* wait for left feeder sensor */
        if (intakeFeederLeft.getFeederDetector()) {
          intakeFeederLeft.setFeederMotor(0);
          testTable.getEntry("Left Feeder Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(20)) {
          intakeFeederLeft.setFeederMotor(0);
          testTable.getEntry("Left Feeder Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 2: /* wait for left intake sensor */
        if (intakeFeederLeft.getIntakeSensor()) {
          intakeFeederLeft.setIntakeMotor(0);
          testTable.getEntry("Left Intake Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(20)) {
          intakeFeederLeft.setIntakeMotor(0);
          testTable.getEntry("Left Intake Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 3: /* purge left */
        intakeFeederLeft.setIntakeMotor(Constants.INTAKE_PURGE_SPEED);
        intakeFeederLeft.setFeederMotor(Constants.FEEDER_PURGE_SPEED);
        if (timer.hasElapsed(0.75)) {
          intakeFeederLeft.setIntakeMotor(0);
          intakeFeederLeft.setFeederMotor(0);
          intakeFeederRight.setIntakeMotor(Constants.INTAKE_TEST_SPEED);
          intakeFeederRight.setFeederMotor(Constants.FEEDER_TEST_SPEED);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 4: /* wait for right feeder sensor */
        if (intakeFeederRight.getFeederDetector()) {
          intakeFeederRight.setFeederMotor(0);
          testTable.getEntry("Right Feeder Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(20)) {
          intakeFeederRight.setFeederMotor(0);
          testTable.getEntry("Right Feeder Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 5: /* wait for right intake sensor */
        if (intakeFeederRight.getIntakeSensor()) {
          intakeFeederRight.setIntakeMotor(0);
          testTable.getEntry("Right Intake Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(20)) {
          intakeFeederRight.setIntakeMotor(0);
          testTable.getEntry("Right Intake Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 6: /* purge right */
        intakeFeederRight.setIntakeMotor(Constants.INTAKE_PURGE_SPEED);
        intakeFeederRight.setFeederMotor(Constants.FEEDER_PURGE_SPEED);
        if (timer.hasElapsed(0.75)) {
          intakeFeederRight.setIntakeMotor(0);
          intakeFeederRight.setFeederMotor(0);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 7: /* drop intake */
        intakeFeederLeft.setPneumaticDown(true);
        intakeFeederRight.setPneumaticDown(true);
        if (timer.hasElapsed(1.5)) {
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 8:
        intakeFeederLeft.setPneumaticDown(false);
        intakeFeederRight.setPneumaticDown(false);
        stage++;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeFeederLeft.setState(State.kOff);
    intakeFeederRight.setState(State.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 9;
  }
}
