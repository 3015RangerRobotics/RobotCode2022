// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class IntakeSetOverride extends InstantCommand {
  int side;
  boolean override;

  /** Creates a new IntakeSetOverride. */
  public IntakeSetOverride(int side, boolean override) {
    this.side = side;
    this.override = override;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeFeeder[side].setPneumaticOverride(override);
  }
}
