// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Sets the hood position */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodSetPosition extends InstantCommand {
  double hoodPos;

  /** Creates a new HoodSetPosition. */
  public HoodSetPosition(double hoodPos) {
    addRequirements(RobotContainer.hood);
    this.hoodPos = hoodPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.hood.setHoodPosition(hoodPos);
  }

}
