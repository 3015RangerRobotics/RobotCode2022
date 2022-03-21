// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.IntakeFeeder.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeStop extends InstantCommand {
  private IntakeFeeder intakeFeeder;

  public IntakeStop(int side) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeFeeder = RobotContainer.intakeFeeder[side];
    addRequirements(intakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeFeeder.setState(State.kOff);
  }
}
