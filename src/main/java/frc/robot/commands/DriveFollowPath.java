// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.Destination;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    Pose2d intialPathPose;
    Pose2d initialPose;

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
        this.path = PathPlanner.loadPath(pathname, 2, 2.5);

        PIDController posController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P,
                Constants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.DRIVE_POS_ERROR_CONTROLLER_D);
        PIDController headingController = new PIDController(Constants.DRIVE_HEADING_ERROR_CONTROLLER_P,
                Constants.DRIVE_HEADING_ERROR_CONTROLLER_I, Constants.DRIVE_HEADING_ERROR_CONTROLLER_D);
        ProfiledPIDController rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P,
                Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY,
                        Constants.DRIVE_MAX_ANGULAR_ACCEL));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.pathController = new SwervePathController(posController, headingController, rotationController);
        this.ignoreHeading = ignoreHeading;            
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intialPathPose = path.getInitialPose();
        initialPose = RobotContainer.drive.getPoseMeters();
        RobotContainer.drive.resetOdometry(
                new Pose2d(RobotContainer.drive.getPoseMeters().getTranslation(), intialPathPose.getRotation()));
        pathController.reset(RobotContainer.drive.getPoseMeters());
        lastTime = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = timer.get();
        PathPlannerState desiredState = (PathPlannerState) path.sample(time);
        desiredState.holonomicRotation = new Rotation2d(-desiredState.holonomicRotation.getRadians());
        desiredState.poseMeters = new Pose2d(desiredState.poseMeters.getTranslation(), new Rotation2d(-desiredState.poseMeters.getRotation().getRadians()));

        if (ignoreHeading)
            desiredState.poseMeters = new Pose2d(desiredState.poseMeters.getTranslation(), new Rotation2d());

        ChassisSpeeds targetSpeeds = pathController.calculate(RobotContainer.drive.getPoseMeters(), desiredState,
                time - lastTime, timer.hasElapsed(0.1));
        RobotContainer.drive.drive(targetSpeeds);

        // Position PID
        // SmartDashboard.putNumber("PIDTarget", 0);
        // SmartDashboard.putNumber("PIDActual", pathController.getPosError());

        // Rotation PID
        // SmartDashboard.putNumber("PIDTarget", desiredState.holonomicRotation.getDegrees());
        // SmartDashboard.putNumber("PIDACtual", RobotContainer.drive.getAngleDegrees());

        // Heading PID
        SmartDashboard.putNumber("PIDTarget", desiredState.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());

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

// public class DriveFollowPath extends CommandBase {
//     Timer timer = new Timer();
//     PathPlannerTrajectory trajectory;
//     HolonomicDriveController controller;

//     public DriveFollowPath(String pathname) {
//         this(pathname, Constants.SWERVE_MAX_VELOCITY_METERS, Constants.SWERVE_MAX_ACCEL_METERS);
//     }

//     public DriveFollowPath(String pathName, double maxVel, double maxAccel) {
//         addRequirements(RobotContainer.drive);

//         this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel);

//         PIDController xController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0);
//         PIDController yController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, 0, 0);
//         ProfiledPIDController thetaController = new ProfiledPIDController(
//                 Constants.DRIVE_ROTATION_CONTROLLER_P, 0, 0,
//                 new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY,
//                         Constants.DRIVE_MAX_ANGULAR_ACCEL));
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);
//         this.controller = new HolonomicDriveController(xController, yController, thetaController);
//     }

//     @Override
//     public void initialize() {
//         timer.reset();
//         timer.start();

//         RobotContainer.drive.resetOdometry(trajectory.getInitialPose());
//     }

//     @Override
//     public void execute(){
//         double time = timer.get();
//         PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);

//         Translation2d translation = desiredState.poseMeters.getTranslation();

//         desiredState.poseMeters = new Pose2d(new Translation2d(translation.getX(), -translation.getY()), desiredState.poseMeters.getRotation());

//         ChassisSpeeds targetSpeeds = controller.calculate(RobotContainer.drive.getPoseMeters(), desiredState, new Rotation2d(-desiredState.holonomicRotation.getRadians()));


//         // Position PID
//         // SmartDashboard.putNumber("PIDTarget", 0);
//         // SmartDashboard.putNumber("PIDActual", pathController.getPosError());

//         // Rotation PID
//         // SmartDashboard.putNumber("PIDTarget", desiredState.holonomicRotation.getDegrees());
//         // SmartDashboard.putNumber("PIDACtual", RobotContainer.drive.getAngleDegrees());

//         // Heading PID
//         // SmartDashboard.putNumber("PIDTarget", desiredState.poseMeters.getRotation().getDegrees());
//         // SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());
//         System.out.println("tr:" + Math.round(desiredState.holonomicRotation.getDegrees()) + ", " + "r:" + Math.round(RobotContainer.drive.getAngleDegrees()) + " | th:" + Math.round(desiredState.poseMeters.getRotation().getDegrees()));


//         RobotContainer.drive.drive(targetSpeeds);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         timer.stop();
//         RobotContainer.drive.drive(0, 0, 0, true);
//     }

//     @Override
//     public boolean isFinished() {
//         return timer.hasElapsed(trajectory.getTotalTimeSeconds());
//     }
// }