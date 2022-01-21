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

    Translation2d[] moduleLocations = new Translation2d[4];
    SwerveModule[] swerveModules = new SwerveModule[4];

    PigeonIMU imu;

    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    /**
     * Creates a new Drive.
     */
    public Drive() {

        moduleLocations[0] = new Translation2d(Constants.SWERVE_CENTER_DISTANCE, Constants.SWERVE_CENTER_DISTANCE); // Front Right
        moduleLocations[1] = new Translation2d(Constants.SWERVE_CENTER_DISTANCE, -Constants.SWERVE_CENTER_DISTANCE); // Front Left
        moduleLocations[2] = new Translation2d(-Constants.SWERVE_CENTER_DISTANCE, -Constants.SWERVE_CENTER_DISTANCE); // Back Left
        moduleLocations[3] = new Translation2d(-Constants.SWERVE_CENTER_DISTANCE, Constants.SWERVE_CENTER_DISTANCE); // Back Right
        
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new SwerveModule(Constants.SWERVE_DRIVE_CHANNELS[i], Constants.SWERVE_ROTATION_CHANNELS[i], 0);
        }

        resetEncoders();

        imu = new PigeonIMU(Constants.DRIVE_PIGEON_CHANNEL);
        imu.configFactoryDefault();
        resetIMU();

        kinematics = new SwerveDriveKinematics(moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, getAngleRotation2d());
    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("gyro", getAngleDegrees());
    }

    /**
     * Reset the encoder position of all swerve modules' drive and rotation sensors
     */
    public void resetEncoders() {
        for (SwerveModule s : swerveModules) {
            s.resetEncoders();
        }
    }

    /**
     * Sets all swerve modules' drive motors to brake or coast mode
     * 
     * @param enable <ul><li>true - Brake mode;
     *               <li>false - Coast mode
     */
    public void enableBrakeMode(boolean enable){
        for(SwerveModule s : swerveModules) {
            s.enableBrakeMode(enable);
        }
    }

    /**
     * Resets the IMU's position such that the current heading will read as '0'
     * 
     */
    public void resetIMU() {
        imu.setFusedHeading(0);
        imu.setYaw(0);
    }

    /**
     * Sets the IMU's position to a specified angle such that the current heading
     * will read as the input angle
     * 
     * @param angle the angle in degrees to set the IMU to
     */
    public void setAngle(double angle) {
        imu.setFusedHeading(angle);
        imu.setYaw(angle);
    }

    /**
     * Gets the IMU's current heading in degrees
     * 
     * @return the IMU's heading in degrees normalized between -180 and +180
     */
    public double getAngleDegrees() {
        double angle = imu.getFusedHeading() % 360;
        if(angle > 180){
            angle -= 360;
        }else if(angle <= -180) {
            angle += 360;
        }
        return -angle;
    }

    /**
     * Gets the Robot's heading as a <code>Rotation2d</code> object
     * 
     * @return a Rotation2d object representing the robot's current heading
     */
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(getAngleDegrees());
    }

    /**
     * Update's the Odometry based on current swerve module states
     */
    public void updateOdometry() {
        odometry.update(getAngleRotation2d(), swerveModules[0].getSwerveModuleState(), swerveModules[1].getSwerveModuleState(), swerveModules[2].getSwerveModuleState(), swerveModules[3].getSwerveModuleState());
    }

    /**
     * Reset's the Odometry based on the input pose
     * 
     * @param pose a <code>Pose2d</code> representing the Robot's new position and angle
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getAngleRotation2d());
    }

    /**
     * Sets the output of all drive motors as a percentage of their maximum output
     * 
     * @param pct a number representing percentage speed normalized between -1 and +1
     */
    public void setModuleDrivePct(double pct) {
        for (SwerveModule s : swerveModules) {
            s.setDriveMotor(ControlMode.PercentOutput, pct);
        }
    }

    /**
     * Sets the rotation of all modules
     * 
     * @param degrees the rotation relative to the absolute zero in degrees
     */
    public void setModuleRotation(double degrees) {
        for (SwerveModule s : swerveModules) {
            s.setRotationPosition(degrees);
        }
    }

    /**
     * Sets the rotation of a module with a specified index
     * 
     * @param degrees the rotation relative to the absolute zero in degrees
     * @param index the index of the motor to rotate;
     *              <ul><li>0 - Front Right
     *              <li>1 - Front Left
     *              <li>2 - Back Left
     *              <li>3 - Back Right
     */
    public void setModuleRotation(double degrees, int index) {
        swerveModules[index].setRotationPosition(degrees);
    }


    /**
     * Gets the rotation of a module with specified index
     * 
     * @param index the index of the module
     *              <ul><li>0 - Front Right
     *              <li>1 - Front Left
     *              <li>2 - Back Left
     *              <li>3 - Back Right
     */
    public double getModuleRotation(int index) {
        return swerveModules[index].getRelativeRotation();
    }

    /**
     * Sets the states of all swerve modules based on input <code>ChassisSpeeds</code>
     * 
     * @param chassisSpeeds an object encapsulating desired dx, dy, and dθ of the robot
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SWERVE_MAX_VELOCITY_METERS);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState currentState = swerveModules[i].getSwerveModuleState();
            SwerveModuleState.optimize(moduleStates[i], currentState.angle);
            swerveModules[i].setSwerveModuleState(moduleStates[i]);
        }
    }

    /**
     * Sets the states of all swerve modules based on input velocities with NWU coordinates
     * 
     * @param xVelMeters the desired velocity of the robot in the x direction in meters; relative to either the robot or the field.
     * @param yVelMeters the desired velocity of the robot in the y direction in meters; relative to either the robot or the field.
     * @param degreesPerSecond the desired angular velocity of the robot in degrees
     * @param isFieldRelative <ul><li>true - the x and y velocities refer to an absolute coordinate axis defined by the field;
     *                        <li>false - the x and y velocities refer to a coordinate axis defined by the robot's current angle
     */
    public void drive(double xVelMeters, double yVelMeters, double degreesPerSecond, boolean isFieldRelative) {
        if (isFieldRelative) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond), getAngleRotation2d()));
        } else {
            drive(new ChassisSpeeds(xVelMeters, yVelMeters, Math.toRadians(degreesPerSecond)));
        }
    }

    /**
     * Gets the robot's current position as a <code>Pose2d</code> with distance units in meters
     * 
     * @return the robot's current Pose in meters
     */
    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the zeros of all swerve modules to their current positions
     */
    public void resetZeros() {
        for (SwerveModule s : swerveModules) {
            s.updateRotationOffset();
        }
    }
}