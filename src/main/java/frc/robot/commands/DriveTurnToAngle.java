// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Turns the Robot to a Target Angle */

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTurnToAngle extends CommandBase {
  double targetAngle;
  ProfiledPIDController rotationController;

  /** Creates a new DriveTurnToAngle. */
  public DriveTurnToAngle(double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P,
        Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
        new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
    rotationController.enableContinuousInput(-180, 180);
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationOutput = rotationController.calculate(RobotContainer.drive.getAngleDegrees(), targetAngle);
    RobotContainer.drive.drive(0, 0, rotationOutput, false);

    SmartDashboard.putNumber("PIDTarget", targetAngle);
    SmartDashboard.putNumber("PIDActual", RobotContainer.drive.getAngleDegrees());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
