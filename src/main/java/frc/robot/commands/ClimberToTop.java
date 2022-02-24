// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Brings the Climber non-floppy arm to topmost extension */ 
/* Part of the climbing Sequential command group */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberToTop extends CommandBase {

  boolean isLoaded = false;

  public ClimberToTop() {
    this(false);
  }

  /**
   * Creates a new ClimberToTop. Moves the climber to its top position. Finishes
   * when the climber is within 0.5cm of the top position.
   */
  public ClimberToTop(boolean isLoaded) {
    this.isLoaded = isLoaded;
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.setPIDSlot(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isLoaded) {
      RobotContainer.climber.setOutput(0.005);
    } else {
      RobotContainer.climber.setClimberPos(Constants.CLIMBER_MAX_HEIGHT_METERS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Constants.CLIMBER_MAX_HEIGHT_METERS
    - RobotContainer.climber.getClimberPos()) <
    Constants.CLIMBER_HEIGHT_TOLERANCE; // 0.5 cm
    // return false;
  }
}
