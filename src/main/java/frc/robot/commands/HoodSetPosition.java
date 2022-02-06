// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodSetPosition extends InstantCommand {
  double hoodPos;
  private Hood hood;

  /** Creates a new HoodSetPosition. */
  public HoodSetPosition(int side, double hoodPos) {
    this.hood = RobotContainer.hood[side];
    addRequirements(hood);
    this.hoodPos = hoodPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setHoodPosition(hoodPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
