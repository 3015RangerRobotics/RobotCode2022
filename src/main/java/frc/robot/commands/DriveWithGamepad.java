// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveWithGamepad extends CommandBase {

  private boolean isFieldRelative;
  private boolean isCounterRotationOn;
  ProfiledPIDController rotationController;
  double currentAngle;
  double previousGyroAngle;
  final double trainingWheels = 0.6;

  /**
   * Command for controlling the robot with the gamepad.
   * 
   * Runs indefinitely.
   */
  public DriveWithGamepad(boolean isFieldRelative, boolean isCounterRotationOn) {
    addRequirements(RobotContainer.drive);
    rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P,
        Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
        new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
    rotationController.enableContinuousInput(-180, 180);
    this.isFieldRelative = isFieldRelative;
    this.isCounterRotationOn = isCounterRotationOn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = RobotContainer.drive.getAngleDegrees();
    previousGyroAngle = currentAngle;
    rotationController.reset(currentAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightStickX = RobotContainer.getDriverRightStickX();
    double leftStickY = RobotContainer.getDriverLeftStickY();
    double leftStickX = RobotContainer.getDriverLeftStickX();

    double rotationOutput = rightStickX;

    if (isCounterRotationOn && Math.abs(rotationOutput) < 0.05) {
      if (Math.abs(RobotContainer.drive.getAngleDegrees() - previousGyroAngle) < 1.8) {
        rotationOutput = rotationController.calculate(RobotContainer.drive.getAngleDegrees(), currentAngle);
      } else {
        currentAngle = RobotContainer.drive.getAngleDegrees();
        rotationController.reset(currentAngle);
      }
      // if ((Math.abs(leftStickX) == 0 && Math.abs(leftStickY) == 0) && RobotContainer.getDriverLeftTrigger() <= 0.5)
      //   rotationOutput = 0;

      // SmartDashboard.putNumber("PIDTarget", currentAngle);
      // SmartDashboard.putNumber("PIDActual",
      // RobotContainer.drive.getAngleDegrees());
    } else {
      currentAngle = RobotContainer.drive.getAngleDegrees();
      rotationController.reset(currentAngle);
      rotationOutput *= Constants.DRIVE_MAX_ANGULAR_VELOCITY;
    }
    previousGyroAngle = RobotContainer.drive.getAngleDegrees();
    double xVel = -leftStickY * Constants.SWERVE_MAX_VELOCITY_METERS * trainingWheels;
    double yVel = leftStickX * Constants.SWERVE_MAX_VELOCITY_METERS * trainingWheels;

    Translation2d corrections = new Translation2d(xVel, yVel);

    RobotContainer.drive.drive(corrections.getX(), corrections.getY(), rotationOutput, isFieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
