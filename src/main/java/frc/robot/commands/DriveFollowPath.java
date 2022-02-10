// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import lib.swerve.SwervePath;
import lib.swerve.SwervePathController;

public class DriveFollowPath extends CommandBase {
    Timer timer;
    Trajectory path;
    SwervePathController pathController;
    double lastTime;
    boolean ignoreHeading;

    /**
     * Drives the robot according to generated path. Ends according to the duration
     * of the path.
     * 
     * @param pathname filepath to .csv representing path to follow
     */
    public DriveFollowPath(String pathname) {
        this(pathname, false);
    }

    /**
     * Drives the robot according to generated path while ignoring the robot's
     * rotation
     * 
     * @param pathname      filepath to .csv representing path to follow
     * @param ignoreHeading
     *                      <ul>
     *                      <li>true - ignores the robot's rotational orientation
     *                      <li>false (default) - controls robot's rotational
     *                      orientation according to path
     */
    public DriveFollowPath(String pathname, boolean ignoreHeading) {
        addRequirements(RobotContainer.drive);
        this.timer = new Timer();
        this.path = PathPlanner.loadPath(pathname, Constants.SWERVE_MAX_VELOCITY_METERS,
                Constants.SWERVE_MAX_ACCEL_METERS);

        PIDController posController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P,
                Constants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.DRIVE_POS_ERROR_CONTROLLER_D);
        PIDController headingController = new PIDController(Constants.DRIVE_HEADING_ERROR_CONTROLLER_P,
                Constants.DRIVE_HEADING_ERROR_CONTROLLER_I, Constants.DRIVE_HEADING_ERROR_CONTROLLER_D);
        ProfiledPIDController rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P,
                Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY,
                        Constants.DRIVE_MAX_ANGULAR_ACCEL));
        this.pathController = new SwervePathController(posController, headingController, rotationController);
        this.ignoreHeading = ignoreHeading;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        Pose2d initialState = path.getInitialPose();
        RobotContainer.drive.resetOdometry(
                new Pose2d(RobotContainer.drive.getPoseMeters().getTranslation(), initialState.getRotation()));
        pathController.reset(RobotContainer.drive.getPoseMeters());
        lastTime = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = timer.get();
        PathPlannerState desiredState = (PathPlannerState) path.sample(time);

        if (ignoreHeading)
            desiredState.poseMeters = new Pose2d(desiredState.poseMeters.getTranslation(), new Rotation2d());

        ChassisSpeeds targetSpeeds = pathController.calculate(RobotContainer.drive.getPoseMeters(), desiredState,
                time - lastTime, timer.hasElapsed(0.1));
        RobotContainer.drive.drive(targetSpeeds);

        lastTime = time;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotContainer.drive.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getTotalTimeSeconds());
    }
}