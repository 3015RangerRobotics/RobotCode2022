// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ClimbSetSubsystemAlwaysActive extends InstantCommand {

  boolean active;

  /** Creates a new ClimbSetSubsystemAlwaysActive. */
  public ClimbSetSubsystemAlwaysActive(boolean active) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.active = active;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.isClimberRunning.setAlwaysActive(active);
  }
}
