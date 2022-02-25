// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Brings Climber non-floppy arm down to robot */ 
/* Part of the Climbing sequential Command Group */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberToBottom extends CommandBase {

  /**
   * Creates a new ClimberToBottom. Moves the climber to its bottom position.
   * Finishes when the climber is within 0.5cm of the bottom
   */
  public ClimberToBottom() {
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.setPIDSlot(1);
    RobotContainer.climber.setBrakeMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.setClimberPos(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.climber.getClimberPos() <
    Constants.CLIMBER_HEIGHT_TOLERANCE; // 0.5 cm
    // return false;
  }
}
