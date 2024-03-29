// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Enables the Neumatic Compressor */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class CompressorSetEnabled extends InstantCommand {
  private boolean enabled;

  /**
   * Creates a new CompressorSetEnabled. Sets the compressor to the parameter of
   * the constructor. Instant command.
   */
  public CompressorSetEnabled(boolean enabled) {
    addRequirements(RobotContainer.compressor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.compressor.setCompressorEnabled(enabled);
  }

}
