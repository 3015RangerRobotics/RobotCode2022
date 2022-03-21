// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Shooter go pew pew with specificed rpm */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeFeeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeFeeder.State;

public class ShootBalls extends CommandBase {
  private IntakeFeeder intakeFeeder;
  private Shooter shooter;
  double rpm;
  double secondBallDelay;
  double initialDelay;
  boolean hasTwoBalls;
  Timer timer;

  public ShootBalls(int side, double rpm) {
    this(side, rpm, 0, 0);
  }

  public ShootBalls(int side, double rpm, double secondBallDelay) {
    this(side, rpm, secondBallDelay, 0);
  }

  /** Creates a new ShootBalls. */
  public ShootBalls(int side, double rpm, double secondBallDelay, double initialDelay) {
    this.intakeFeeder = RobotContainer.intakeFeeder[side];
    // this.shooter = RobotContainer.shooter[side];
    this.rpm = rpm;
    this.timer = new Timer();
    this.initialDelay = initialDelay;
    this.secondBallDelay = secondBallDelay;
    addRequirements(intakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasTwoBalls = intakeFeeder.getIntakeSensor() && intakeFeeder.getFeederDetector();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = timer.get();
    if (time > initialDelay) {
      intakeFeeder.setState(State.kShootFeeder);
    }
    if (hasTwoBalls && time > initialDelay + secondBallDelay) {
      intakeFeeder.setState(State.kShootFeederIntake);
    } else if (!hasTwoBalls && time > initialDelay) {
      intakeFeeder.setState(State.kShootFeederIntake);
    }
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
