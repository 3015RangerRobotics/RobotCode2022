// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Intakes a game piece */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.IntakeFeeder.State;

public class IntakeBall extends CommandBase {
  private IntakeFeeder intakeFeeder;
  private boolean affectPneumatic;

  public IntakeBall(int side) {
    this(side, true);
  }

  /** Creates a new IntakeBall. Will run eternally */
  public IntakeBall(int side, boolean affectPneumatic) {
    intakeFeeder = RobotContainer.intakeFeeder[side];
    this.affectPneumatic = affectPneumatic;
    addRequirements(intakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeFeeder.setState(State.kFillToFeeder);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeFeeder.setState(State.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
