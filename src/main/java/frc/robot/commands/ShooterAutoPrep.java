// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Sets the rpm and hood based on limelight but doesn't shoot */

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShooterAutoPrep extends CommandBase {

  double speed;
  double angle;
  boolean hasHadTarget = false;
  Pose2d previousPose;

  /** Creates a new ShooterAutoPrep. */
  public ShooterAutoPrep() {
    addRequirements(RobotContainer.hood, RobotContainer.shooter[0], RobotContainer.shooter[1]);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.checkout();
    previousPose = RobotContainer.drive.getPoseMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.limelight.hasTarget()) {
      speed = RobotContainer.limelight.getShooterSpeed();
      angle = RobotContainer.limelight.getHoodPos();
      hasHadTarget = true;
    }
    double differencce = previousPose.minus(RobotContainer.drive.getPoseMeters()).getTranslation().getNorm();
    if (hasHadTarget && !(RobotContainer.driverRT.get() || RobotContainer.coDriverRT.get()) && differencce > 0 && RobotContainer.hood.hasBeenHomed()) {
      previousPose = RobotContainer.drive.getPoseMeters();
      RobotContainer.hood.setHoodPosition(angle);
      RobotContainer.shooter[0].setRPM(speed);
      RobotContainer.shooter[1].setRPM(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.limelight.uncheckout();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
