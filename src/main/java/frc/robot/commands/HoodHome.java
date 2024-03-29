// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Brings the hood to its zero position */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodHome extends CommandBase {
  double speed;

  public HoodHome() {
    this(-0.8);
  }

  /** Creates a new HoodHome. */
  public HoodHome(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    this.speed = -Math.abs(speed);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.hood.setReverseLimit(false);
    if (!RobotContainer.hood.hasBeenHomed()) {
      RobotContainer.hood.setHoodOutputPercentage(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotContainer.hood.hasBeenHomed()) {
      RobotContainer.hood.setHoodOutputPercentage(0);
    }
    RobotContainer.hood.setReverseLimit(true);
    if (RobotContainer.hood.getReverseLimit()) {
      RobotContainer.hood.resetZero();
      RobotContainer.hood.setHomed();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.hood.getReverseLimit() || RobotContainer.hood.hasBeenHomed();
  }
}
