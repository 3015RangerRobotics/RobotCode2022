// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.FloatSerializer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ShooterSetSpeedOverride extends InstantCommand {
  boolean enabled;
  /** Creates a new ShooterSetSpeedOverride. */
  public ShooterSetSpeedOverride(boolean enabled) {
    this.enabled = enabled;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter[0].setMinSpeedOverride(enabled);
    RobotContainer.shooter[1].setMinSpeedOverride(enabled);
    if (enabled) {
      RobotContainer.shooter[0].stop();
      RobotContainer.shooter[1].stop();
    }
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
