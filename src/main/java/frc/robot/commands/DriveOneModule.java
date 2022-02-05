// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOneModule extends CommandBase {
  //TODO:make this command, a simple testing command to drive and run one module, be inspired by: https://github.com/Greater-Rochester-Robotics/Swerve2021-340/blob/master/src/main/java/frc/robot/commands/Drive/util/DriveOneModule.java
  /** Creates a new DriveOneModule. */
  public DriveOneModule() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
