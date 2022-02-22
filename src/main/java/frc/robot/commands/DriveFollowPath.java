// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveFollowPath extends NewServeControllerCommand {

  public static PIDController xPositionController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0);
  public static PIDController yPositionController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0);
  public static ProfiledPIDController thetaController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, 0, 0,
      new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));

  public DriveFollowPath(String pathname) {
    this(pathname, Constants.SWERVE_MAX_VELOCITY_METERS, Constants.SWERVE_MAX_ACCEL_METERS);
  }

  /** Creates a new DriveFollowPath. */
  public DriveFollowPath(String pathname, double maxVel, double maxAccel) {
    super(PathPlanner.loadPath(pathname, maxVel, maxAccel), RobotContainer.drive::getPoseMeters,
        RobotContainer.drive.getKinematics(), xPositionController, yPositionController, thetaController,
        RobotContainer.drive::setAllModuleStates, RobotContainer.drive);
    thetaController.enableContinuousInput(-180, 180);
  }
}
