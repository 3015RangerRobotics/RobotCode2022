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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.SolenoidPosition;

public class TestClimber extends CommandBase {

  NetworkTable testTable = NetworkTableInstance.getDefault().getTable("test");

  Climber climber;
  int stage = 0;
  boolean result = true;

  double[] testPositions = {0, 0.5, 0.25, 0.75, 1};

  Timer timer;
  /** Creates a new TestClimber. */
  public TestClimber() {

    climber = RobotContainer.climber;

    addRequirements(climber);

    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
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
    switch(stage) {
      case 0: /* start homing */
        if (climber.getBottomLimit()) {
          stage++;
          break;
        }
        climber.setOutput(-0.3);
        stage++;
        break;
      case 1: /* home climber */
        if (climber.getBottomLimit()) {
          testTable.getEntry("Climber home test").setBoolean(true);
          climber.setOutput(0);
          climber.setSensorZero();
          timer.reset();
          timer.start();
          stage++;
        } else if (timer.hasElapsed(4)) {
          testTable.getEntry("Climber home test").setBoolean(false);
          climber.setOutput(0);
          stage = 8;
        } else if (climber.getClimberCurrent() > 10) {
          testTable.getEntry("Climber home test").setBoolean(false);
          climber.setOutput(0);
          climber.setSensorZero();
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        climber.setClimberPos(testPositions[stage - 2] * Constants.CLIMBER_MAX_HEIGHT_METERS);
        if (timer.hasElapsed(2)) {
          result &= Math.abs(climber.getClimberPos() - (testPositions[stage - 2] * Constants.CLIMBER_MAX_HEIGHT_METERS)) < 0.01;
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 7:
        testTable.getEntry("Climber position test").setBoolean(result);
        stage++;
        break;
      case 8:
        if(RobotContainer.driverStart.get() && RobotContainer.driverBack.get()) {
          timer.reset();
          timer.start();
          stage++;
        }
        break;
      case 9:
        climber.setSolenoidPosition(SolenoidPosition.kUp);
        if (timer.hasElapsed(0.8)) {
          stage++;
        }
        break;
      case 10:
        if(RobotContainer.driverStart.get() && RobotContainer.driverBack.get()) {
          timer.reset();
          timer.start();
          stage++;
        }
        climber.setSolenoidPosition(SolenoidPosition.kDown);
        if (timer.hasElapsed(0.8)) {
          stage++;
        }
        break;
      case 11:
        testTable.getEntry("Floppy arm test completed").setBoolean(true);
        stage++;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 12;
  }
}
