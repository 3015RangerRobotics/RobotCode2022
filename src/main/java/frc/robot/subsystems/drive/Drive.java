// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

    Translation2d frontLeftLocation = new Translation2d(Constants.SWERVE_CENTER_DISTANCE, -Constants.SWERVE_CENTER_DISTANCE);
    Translation2d frontRightLocation = new Translation2d(Constants.SWERVE_CENTER_DISTANCE, Constants.SWERVE_CENTER_DISTANCE);
    Translation2d backLeftLocation = new Translation2d(-Constants.SWERVE_CENTER_DISTANCE, -Constants.SWERVE_CENTER_DISTANCE);
    Translation2d backRightLocation = new Translation2d(-Constants.SWERVE_CENTER_DISTANCE, Constants.SWERVE_CENTER_DISTANCE);

    SwerveModule frontLeftSwerveModule;
    SwerveModule frontRightSwerveModule;
    SwerveModule backLeftSwerveModule;
    SwerveModule backRightSwerveModule;

    PigeonIMU imu;

    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    /**
     * Creates a new Drive.
     */
    public Drive() {
        frontLeftSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_FL, Constants.SWERVE_ROTATION_CHANNEL_FL, 140);
        frontRightSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_FR, Constants.SWERVE_ROTATION_CHANNEL_FR, 56);
        backLeftSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_BL, Constants.SWERVE_ROTATION_CHANNEL_BL, -117);
        backRightSwerveModule = new SwerveModule(Constants.SWERVE_DRIVE_CHANNEL_BR, Constants.SWERVE_ROTATION_CHANNEL_BR, 72);
        resetEncoders();

        imu = new PigeonIMU(Constants.DRIVE_PIGEON_CHANNEL);
        imu.configFactoryDefault();
        resetIMU();

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(kinematics, getAngleRotation2d());
    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("gyro", getAngleDegrees());
    }

    public void resetEncoders() {
        frontLeftSwerveModule.resetEncoders();
        frontRightSwerveModule.resetEncoders();
        backLeftSwerveModule.resetEncoders();
        backRightSwerveModule.resetEncoders();
    }

    public void enableBrakeMode(boolean enable){
        frontLeftSwerveModule.enableBrakeMode(enable);
        frontRightSwerveModule.enableBrakeMode(enable);
        backLeftSwerveModule.enableBrakeMode(enable);
        backRightSwerveModule.enableBrakeMode(enable);
    }

    public void resetIMU() {
        imu.setFusedHeading(0);
        imu.setYaw(0);
    }

    public void setAngle(double angle) {
        imu.setFusedHeading(angle);
        imu.setYaw(angle);
    }

    public double getAngleDegrees() {
        double angle = imu.getFusedHeading() % 360;
        if(angle > 180){
            angle -= 360;
        }else if(angle <= -180) {
            angle += 360;
        }
        return -angle;
    }

    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(getAngleDegrees());
    }

    public void updateOdometry() {
        odometry.update(getAngleRotation2d(), frontLeftSwerveModule.getSwerveModuleState(), frontRightSwerveModule.getSwerveModuleState(), backLeftSwerveModule.getSwerveModuleState(), backRightSwerveModule.getSwerveModuleState());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getAngleRotation2d());
    }

    public void setModuleDrivePct(double pct) {
        frontLeftSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
        frontRightSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
        backLeftSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
        backRightSwerveModule.setDriveMotor(ControlMode.PercentOutput, pct);
    }

    public void setModuleRotation(double degrees) {
        frontLeftSwerveModule.setRotationPosition(degrees);
        frontRightSwerveModule.setRotationPosition(degrees);
        backLeftSwerveModule.setRotationPosition(degrees);
        backRightSwerveModule.setRotationPosition(degrees);
    }
    public void setFrontLeftRotation(double degrees) {
        frontLeftSwerveModule.setRotationPosition(degrees);
    }
    public void setFrontRightRotation(double degrees) {
        frontRightSwerveModule.setRotationPosition(degrees);
    }
    public void setBackLeftRotation(double degrees) {
        backLeftSwerveModule.setRotationPosition(degrees);
    }
    public void setBackRightRotation(double degrees) {
        backRightSwerveModule.setRotationPosition(degrees);
    }

    public double getFrontLeftRotation() {
        return frontLeftSwerveModule.getRelativeRotation();
    }

    public double getFrontRightRotation() {
        return frontRightSwerveModule.getRelativeRotation();
    }

    public double getBackLeftRotation() {
        return backLeftSwerveModule.getRelativeRotation();
    }

    public double getBackRightRotation() {
        return backRightSwerveModule.getRelativeRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        frontLeftSwerveModule.setSwerveModuleState(moduleStates[0]);
        frontRightSwerveModule.setSwerveModuleState(moduleStates[1]);
        backLeftSwerveModule.setSwerveModuleState(moduleStates[2]);
        backRightSwerveModule.setSwerveModuleState(moduleStates[3]);
    }

    public void drive(double xVelMeters, double yVelMeters, double degreesPerSecond, boolean isFieldRelative) {
        if (isFieldRelative) {

            drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond), getAngleRotation2d()));
        } else {
            drive(new ChassisSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond)));
        }
    }

    public void driveFieldRelativeCheese(double xVelMeters, double yVelMeters, double degreesPerSecond, double rotOffset){
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond), getAngleRotation2d().plus(Rotation2d.fromDegrees(rotOffset))));
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }
}