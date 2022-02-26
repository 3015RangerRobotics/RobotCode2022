// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeSolenoidPosition;

public class IntakeSetPneumatic extends InstantCommand {
  Intake.IntakeSolenoidPosition position;

  /** Creates a new IntakeSetPneumatic. */
  public IntakeSetPneumatic(Intake.IntakeSolenoidPosition position) {
    addRequirements(RobotContainer.intake[0]);
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake[0].setPneumaticPosition(position);
  }
}
