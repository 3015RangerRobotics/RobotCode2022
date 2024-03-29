// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class IntakeSetPneumatic extends InstantCommand {
  int side;
  boolean intakeDown;

  /** Creates a new IntakeSetPneumatic. */
  public IntakeSetPneumatic(int side, boolean intakeDown) {
    this.side = side;
    this.intakeDown = intakeDown;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeFeeder[side].setPneumaticDown(intakeDown);
  }
}
