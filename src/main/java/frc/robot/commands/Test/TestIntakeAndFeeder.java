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
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class TestIntakeAndFeeder extends CommandBase {

  NetworkTable testTable = NetworkTableInstance.getDefault().getTable("test");

  Intake intakeLeft;
  Intake intakeRight;
  Feeder feederLeft;
  Feeder feederRight;
  Timer timer;

  int stage = 0;

  /** Creates a new TestIntake. */
  public TestIntakeAndFeeder() {
    intakeLeft = RobotContainer.intake[0];
    intakeRight = RobotContainer.intake[1];
    feederLeft = RobotContainer.feeder[0];
    feederRight = RobotContainer.feeder[1];
    addRequirements(intakeLeft, intakeRight, feederLeft, feederRight);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stage) {
      case 0: /* runs left intake and feeder */
        intakeLeft.test();
        feederLeft.setPercentOutput(Constants.FEEDER_TEST_SPEED);
        stage++;
        break;
      case 1: /* wait for left feeder sensor */
        if (feederLeft.getBallDetector()) {
          feederLeft.setPercentOutput(0);
          testTable.getEntry("Left Feeder Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(5)) {
          feederLeft.setPercentOutput(0);
          testTable.getEntry("Left Feeder Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 2: /* wait for left intake sensor */
        if (intakeLeft.getIntakeSensor()) {
          intakeLeft.stop();
          testTable.getEntry("Left Intake Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(5)) {
          intakeLeft.stop();
          testTable.getEntry("Left Intake Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 3: /* purge left */
        intakeLeft.purge();
        feederLeft.setPercentOutput(Constants.FEEDER_PURGE_SPEED);
        if (timer.hasElapsed(0.75)) {
          intakeLeft.stop();
          feederLeft.setPercentOutput(0);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 4: /* wait for right feeder sensor */
        if (feederRight.getBallDetector()) {
          feederRight.setPercentOutput(0);
          testTable.getEntry("Right Feeder Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(5)) {
          feederRight.setPercentOutput(0);
          testTable.getEntry("Right Feeder Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 5: /* wait for right intake sensor */
        if (intakeRight.getIntakeSensor()) {
          intakeRight.stop();
          testTable.getEntry("Right Intake Sensor").setBoolean(true);
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(5)) {
          intakeRight.stop();
          testTable.getEntry("Right Intake Sensor").setBoolean(false);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 6: /* purge right */
        intakeRight.purge();
        feederRight.setPercentOutput(Constants.FEEDER_PURGE_SPEED);
        if (timer.hasElapsed(0.75)) {
          intakeRight.stop();
          feederRight.setPercentOutput(0);
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 7: /* drop intake */
        intakeLeft.setPneumaticPosition(Intake.IntakeSolenoidPosition.kDown);
        if (timer.hasElapsed(1)) {
          timer.reset();
          timer.start();
          stage++;
        }
      case 8:
        intakeLeft.setPneumaticPosition(Intake.IntakeSolenoidPosition.kUp);
        stage++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeLeft.stop();
    intakeRight.stop();
    feederLeft.setPercentOutput(0);
    feederRight.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 7;
  }
}
