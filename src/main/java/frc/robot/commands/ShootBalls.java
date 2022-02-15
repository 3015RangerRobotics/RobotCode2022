// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootBalls extends CommandBase {
  private Intake intake;
  private Feeder feeder;
  private Shooter shooter;
  double rpm;
  double loopDelay;
  double initialDelay;
  Timer timer;

  public ShootBalls(int side, double rpm) {
    this(side, rpm, 0, 0);
  }

  /** Creates a new ShootBalls. */
  public ShootBalls(int side, double rpm, double loopDelay, double initialDelay) {
    this.intake = RobotContainer.intake[side];
    this.feeder = RobotContainer.feeder[side];
    this.shooter = RobotContainer.shooter[side];
    this.rpm = rpm;
    this.timer = new Timer();
    this.loopDelay = loopDelay < 0.25 ? 0.25 : loopDelay;
    addRequirements(intake, feeder, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.setRPM(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRPM(rpm);
    double time = timer.get() - initialDelay;
    if (time < 0) {
      return;
    }
    double cycle = timer.get() % loopDelay;
    if (cycle < 0.25) {
      feeder.setPercentOutput(Constants.FEEDER_SHOOT_SPEED);
      intake.intake();
    } else {
      feeder.setPercentOutput(0);
      intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    shooter.stop();
    feeder.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
