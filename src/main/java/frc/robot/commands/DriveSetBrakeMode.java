// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class DriveSetBrakeMode extends InstantCommand {

  boolean brakeMode;
  /** Creates a new DriveCoast. */
  public DriveSetBrakeMode(boolean enabled) {
    addRequirements(RobotContainer.drive);

    this.brakeMode = enabled;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drive.enableBrakeMode(brakeMode);
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
