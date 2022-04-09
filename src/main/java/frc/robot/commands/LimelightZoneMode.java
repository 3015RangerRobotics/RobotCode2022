// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class LimelightZoneMode extends InstantCommand {
    boolean zoneMode;

  /** Creates a new LimelightZoneMode. */
  public LimelightZoneMode(boolean zoneMode) {
      this.zoneMode = zoneMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      RobotContainer.limelight.setZoneMode(zoneMode);
  }
}
