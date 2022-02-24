// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Turns the Robot based on the Limelight */

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.LEDMode;

public class DriveTurnToLimelight extends CommandBase {
  double setPoint;
  PIDController rotationController;
  Timer timer;
  boolean hasHadTarget;
  int onTargetCount = 0;

  /** Creates a new DriveTurnToLimelight. */
  public DriveTurnToLimelight() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    rotationController = new PIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I,
        Constants.DRIVE_ROTATION_CONTROLLER_D);
    rotationController.setTolerance(0.5);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.checkout();
    timer.start();
    timer.reset();
    hasHadTarget = false;
    setPoint = RobotContainer.drive.getAngleDegrees();
    onTargetCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Targeting Error", rotationController.getPositionError());
    if (RobotContainer.limelight.hasTarget()) {
      setPoint = RobotContainer.drive.getAngleDegrees() + RobotContainer.limelight.getCorrectedAngleX();
    } else if (RobotContainer.drive.hasOdometryBeenSet() &&
        !hasHadTarget && timer.hasElapsed(.2) && !timer.hasElapsed(1)) {
      // Translation2d relativePos = RobotContainer.drive.getPoseMeters().getTranslation()
      //     .minus(Constants.DRIVE_GOAL_POSITION);
      // setPoint = -1 * Math.toDegrees(Math.atan2(relativePos.getY(), relativePos.getX()))
      //     + RobotContainer.drive.getAngleDegrees();
      if (setPoint > 180) {
        setPoint -= 360;
      } else if (setPoint <= -180) {
        setPoint += 360;
      }
    }

    // SmartDashboard.putBoolean("hasOdometryBeenSet",
    // RobotContainer.drive.hasOdometryBeenSet());

    double output = rotationController.calculate(RobotContainer.drive.getAngleDegrees(), setPoint);
    // if (Math.abs(output) < Constants.DRIVE_ROTATION_MIN_VELOCITY) {
    // output = Constants.DRIVE_ROTATION_MIN_VELOCITY * Math.signum(output);
    // }
    RobotContainer.drive.drive(0.0, 0.0, output, true);
    hasHadTarget = RobotContainer.limelight.hasTarget();
    if (RobotContainer.limelight.hasTarget() && Math.abs(setPoint) < Constants.LL_TOLERANCE) {
      onTargetCount++;
    } else {
      onTargetCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.limelight.uncheckout();
    RobotContainer.drive.drive(0.0, 0.0, 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetCount >= 10;
  }
}
