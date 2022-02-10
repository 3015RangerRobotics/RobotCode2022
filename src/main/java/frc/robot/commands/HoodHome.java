// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodHome extends CommandBase {
  private Hood hood;

  /** Creates a new HoodHome. */
  public HoodHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setHoodOutputPercentage(-0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setHoodOutputPercentage(0);
    if (!interrupted) {
      hood.resetZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.getReverseLimit();
  }
}
