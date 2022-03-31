// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Purges a ball from the intake */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeFeeder.State;

public class PurgeBall extends CommandBase {
  private IntakeFeeder intakeFeeder;
  public Shooter shooter;
  //TODO:finish adding shooter. shooter subsytem might need to add a weak purge method. drive it w/percent output
  /**
   * Creates a new PurgeBall.
   * Runs both the intake and feeder of a specified side in reverse
   * runs indefinitely.
   */
  public PurgeBall(int side) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.intake[side], RobotContainer.feeder[side],
    // RobotContainer.shooter[side]);
    intakeFeeder = RobotContainer.intakeFeeder[side];
    // shooter = RobotContainer.shooter[side];
    addRequirements(intakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeFeeder.setState(State.kPurgeFeederIntake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.purgeShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeFeeder.setState(State.kOff);
    // shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
