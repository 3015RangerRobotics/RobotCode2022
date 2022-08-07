// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSetColorOverride extends InstantCommand {
  int side;
  boolean override;

  public IntakeSetColorOverride(int side, boolean override) {
      this.side = side;
      this.override = override;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeFeeder[side].setColorOverride(override);
  }
}
