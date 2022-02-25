// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Sets the hood position */

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodSetByNetwork extends InstantCommand {
  double hoodPos;

  /** Creates a new HoodSetPosition. */
  public HoodSetByNetwork() {
    addRequirements(RobotContainer.hood);
    this.hoodPos = SmartDashboard.getNumber("Hood Set Position", 0);
    SmartDashboard.putNumber("Hood Set Position", hoodPos);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.hoodPos = SmartDashboard.getNumber("Hood Set Position", 0);
    RobotContainer.hood.setHoodPosition(hoodPos);
  }

}
