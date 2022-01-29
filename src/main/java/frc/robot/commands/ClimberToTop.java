// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberToTop extends CommandBase {

  /** Creates a new ClimberToTop. */
  public ClimberToTop() {
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.setClimberPos(Constants.CLIMBER_MAX_HEIGHT_METERS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.CLIMBER_MAX_HEIGHT_METERS - RobotContainer.climber.getClimberPos() < 0.005; // 0.5 cm
  }
}
