// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DebugAll extends CommandBase {
  /** Creates a new DebugAll. */
  public DebugAll() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.setDebugMode(true);
    RobotContainer.compressor.setDebugMode(true);
    RobotContainer.drive.setDebugMode(true);
    RobotContainer.hood.setDebugMode(true);
    RobotContainer.intakeFeeder[0].setDebugMode(true);
    RobotContainer.intakeFeeder[1].setDebugMode(true);
    RobotContainer.limelight.setDebugMode(true);
    RobotContainer.shooter[0].setDebugMode(true);
    RobotContainer.shooter[1].setDebugMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setDebugMode(false);
    RobotContainer.compressor.setDebugMode(false);
    RobotContainer.drive.setDebugMode(false);
    RobotContainer.hood.setDebugMode(false);
    RobotContainer.intakeFeeder[0].setDebugMode(false);
    RobotContainer.intakeFeeder[1].setDebugMode(false);
    RobotContainer.limelight.setDebugMode(false);
    RobotContainer.shooter[0].setDebugMode(false);
    RobotContainer.shooter[1].setDebugMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
