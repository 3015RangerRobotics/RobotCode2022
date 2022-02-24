// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Moves the non-floppy arm to a distance below top position in order to release bar */
/* Part of the Sequential Command Group for Climbing */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberToBarRelease extends CommandBase {

  /**
   * Creates a new ClimberToBottom. Moves the climber to its bottom position.
   * Finishes when the climber is within 0.5cm of the bottom
   */
  public ClimberToBarRelease() {
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.setPIDSlot(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.setClimberPos(Constants.CLIMBER_RELEASE_HEIGHT_METERS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.climber.getClimberPos() - (Constants.CLIMBER_RELEASE_HEIGHT_METERS)) <
    Constants.CLIMBER_HEIGHT_TOLERANCE; // 0.5 cm
    // return false;
  }
}
