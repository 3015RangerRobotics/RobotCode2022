// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.LEDMode;

public class ShooterAutoShoot extends CommandBase {
  Timer timer = new Timer();
  double loopDelay;
  double initialDelay;

  public ShooterAutoShoot() {
    this(0, 0);
  }

  public ShooterAutoShoot(double loopDelay) {
    this(loopDelay, 0);
  }

  /** Creates a new ShooterPrep. */
  public ShooterAutoShoot(double loopDelay, double initialDelay) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.limelight, RobotContainer.shooter[0], RobotContainer.shooter[1],
        RobotContainer.hood, RobotContainer.feeder[0], RobotContainer.feeder[1]);
    this.loopDelay = loopDelay;
    this.initialDelay = initialDelay;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.checkout();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.limelight.getShooterSpeed();
    double angle = RobotContainer.limelight.getHoodPos();
    RobotContainer.hood.setHoodPosition(angle);
    RobotContainer.shooter[0].setRPM(speed);
    RobotContainer.shooter[1].setRPM(speed);
    RobotContainer.intake[0].intake();
    RobotContainer.intake[1].intake();
    if (RobotContainer.limelight.hasTarget() && RobotContainer.shooter[0].isPrimed(speed)
        && RobotContainer.shooter[1].isPrimed(speed)) {

      double time = timer.get() - initialDelay;
      if (time < 0) {
        return;
      }
      double cycle = timer.get() % loopDelay;
      if (cycle < 0.25) {
        RobotContainer.feeder[0].setPercentOutput(Constants.FEEDER_SHOOT_SPEED);
        RobotContainer.feeder[1].setPercentOutput(Constants.FEEDER_SHOOT_SPEED);
      } else {
        RobotContainer.feeder[0].setPercentOutput(0);
        RobotContainer.feeder[1].setPercentOutput(0);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.limelight.uncheckout();
    RobotContainer.shooter[0].stop();
    RobotContainer.shooter[1].stop();
    RobotContainer.feeder[0].setPercentOutput(0);
    RobotContainer.feeder[1].setPercentOutput(1);
    RobotContainer.intake[0].stop();
    RobotContainer.intake[1].stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
