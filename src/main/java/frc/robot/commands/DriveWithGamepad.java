// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Allows the robot to Drive with the Gamepad */

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C.Port;
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
  final double trainingWheels = 0.85;
  DefaultWheelStates defaultWheelStates;

  public static class DefaultWheelStates {
    double[] positions;
    boolean[] brakes;

    public DefaultWheelStates(double[] positions, boolean[] brakes) {
      this.positions = positions;
      this.brakes = brakes;
    }

    public DefaultWheelStates(double[] positions) {
      this.positions = positions;
    }

    public DefaultWheelStates(boolean[] brakes) {
      this.brakes = brakes;
    }
  }

  /**
   * Command for controlling the robot with the gamepad.
   * 
   * Runs indefinitely.
   */
  public DriveWithGamepad(boolean isFieldRelative, boolean isCounterRotationOn) {
    this(isFieldRelative, isCounterRotationOn, null);
  }

  /**
   * Command for controlling the robot with the gamepad.
   * 
   * Runs indefinitely.
   */
  public DriveWithGamepad(boolean isFieldRelative, boolean isCounterRotationOn, DefaultWheelStates defaultWheelStates) {
    addRequirements(RobotContainer.drive);
    rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P,
        Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
        new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
    rotationController.enableContinuousInput(-180, 180);
    this.isFieldRelative = isFieldRelative;
    this.isCounterRotationOn = isCounterRotationOn;
    this.defaultWheelStates = defaultWheelStates;
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

    // System.out.println(rightStickX);

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
    if (leftStickX == 0 && leftStickY == 0 && Math.abs(rotationOutput) < 0.05 && defaultWheelStates != null) {
      if (defaultWheelStates.positions != null) {
        for (int i = 0; i < 4; i++) {
          RobotContainer.drive.setModuleRotation(defaultWheelStates.positions[i], i);
        }
        if (defaultWheelStates.brakes != null) {
          RobotContainer.drive.setBrakeModes(defaultWheelStates.brakes);
        }
      }
    } else if (defaultWheelStates != null && defaultWheelStates.brakes != null) {
      RobotContainer.drive.setBrakeModes(new boolean[]{false, true, false, true});
    }
    previousGyroAngle = RobotContainer.drive.getAngleDegrees();
    double xVel = -leftStickY * Constants.SWERVE_MAX_VELOCITY_METERS * trainingWheels;
    double yVel = leftStickX * Constants.SWERVE_MAX_VELOCITY_METERS * trainingWheels;

    Translation2d corrections = new Translation2d(xVel, yVel);

    if (Math.abs(corrections.getX()) > 0 || Math.abs(corrections.getY()) > 0 || Math.abs(rotationOutput) > 0 || defaultWheelStates == null) {
      RobotContainer.drive.drive(corrections.getX(), corrections.getY(), rotationOutput, isFieldRelative);
    }
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
