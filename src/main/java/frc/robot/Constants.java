// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* CTRE CAN IDs */
    /* Drive */;
    public static final int[] SWERVE_DRIVE_CHANNELS = { 4, 15, 17, 2 }; // { FR, FL, BL, BR }
    public static final int[] SWERVE_ROTATION_CHANNELS = { 3, 16, 18, 1 }; // { FR, FL, BL, BR }
    public static final int DRIVE_PIGEON_CHANNEL = 0;

    /* Intake */
    public static final int[] INTAKE_MOTORS = { 14, 5 }; // { left, right }

    /* Feeder */
    public static final int[] FEEDER_TOP_MOTORS = { 12, 7 }; // { left, right }
    public static final int[] FEEDER_BOTTOM_MOTORS = { 13, 6 }; // { left, right }

    /* Shooter */
    public static final int[] SHOOTER_MOTORS = { 10, 9 }; // { left, right }

    /* Climber */
    public static final int CLIMBER_MOTOR = 19;

    /* REV ID's */
    /* Hood */
    public static final int[] HOOD_MOTORS = { 11, 8 }; // { left, right }

    /* Solenoid ID's */
    /* Climber */
    public static final int CLIMBER_SOLENOID_UP = 0; // TEMP
    public static final int CLIMBER_SOLENOID_DOWN = 1; // TEMP

    /* Digital IO ID's */
    /* climber */
    public static final int CLIMBER_BOTTOM_SWITCH = 0;
    public static final int CLIMBER_TOP_SWITCH = 0;
    public static final int CLIMBER_PARM_SWITCH = 0;
    public static final int CLIMBER_SARM_SWITCH = 0;
    public static final int CLIMBER_BEAMBREAK_SENSOR = 0;

    /* -------------- CONSTANTS -------------- */
    /* Drive Constants */
    public static final double SWERVE_DRIVE_P = 0;
    public static final double SWERVE_DRIVE_I = 0;
    public static final double SWERVE_DRIVE_D = 0;
    public static final double SWERVE_DRIVE_F = 0;

    public static final double SWERVE_ROTATION_P = 0;
    public static final double SWERVE_ROTATION_I = 0;
    public static final double SWERVE_ROTATION_D = 0;
    public static final double SWERVE_ROTATION_F = 0;

    public static final double DRIVE_ROTATION_CONTROLLER_P = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_F = 0;

    public static final double DRIVE_POS_ERROR_CONTROLLER_P = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_F = 0;

    public static final double DRIVE_HEADING_ERROR_CONTROLLER_P = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_D = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_F = 0;

    public static final double DRIVE_MAX_ANGULAR_ACCEL = 0;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 0;

    public static final double SWERVE_ROTATION_I_ZONE = 0;

    public static final double SWERVE_MAX_VELOCITY_METERS = 0;
    public static final double SWERVE_METERS_PER_PULSE = 0;
    public static final double SWERVE_DEGREES_PER_PULSE = 0;

    public static final double SWERVE_CENTER_DISTANCE = 0;

    /* Shooter Constants */
    public static final double SHOOTER_MOTOR_SPEED = 0;

    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_F = 0;

    public static final double SHOOTER_SHOOT_P = 0;
    public static final double SHOOTER_SHOOT_I = 0;
    public static final double SHOOTER_SHOOT_D = 0;
    public static final double SHOOTER_SHOOT_F = 0;

    public static final double SHOOTER_PULSES_PER_ROTATION = 0;

    /* Limelight Constants */
    public static final double LL_TARGET_HEIGHT = 0;
    public static final double LL_MOUNT_HEIGHT = 0;
    public static final double LL_MOUNT_ANGLE = 0;

    /* Climber Constants */
    public static final double CLIMBER_METERS_PER_PULSE = 0;
    public static final double CLIMBER_MAX_HEIGHT_METERS = 0;
    public static final double CLIMBER_ARM_DEGREES_PER_PULSE = 0;

    public static final double CLIMBER_RAISE_P = 0;
    public static final double CLIMBER_RAISE_I = 0;
    public static final double CLIMBER_RAISE_D = 0;
    public static final double CLIMBER_RAISE_F = 0;

    public static final double CLIMBER_LOWER_P = 0;
    public static final double CLIMBER_LOWER_I = 0;
    public static final double CLIMBER_LOWER_D = 0;
    public static final double CLIMBER_LOWER_F = 0;

    public static final int ENCODER_INPUT_A = 0;
    public static final int ENCODER_INPUT_B = 0;

    /* Hood Constants */
    public static final double HOOD_CONTROLLER_P = 0;
    public static final double HOOD_CONTROLLER_I = 0;
    public static final double HOOD_CONTROLLER_D = 0;
    public static final double HOOD_CONTROLLER_F = 0;
    public static final double HOOD_DEGREES_PER_ROTATION = 1;

    /* Feeder Constants */
    public static final int[] FEEDER_BALL_DETECTORS = { 0, 1 }; // { left, right }
    public static final double FEEDER_INTAKE_SPEED = 1;
    public static final double FEEDER_PURGE_SPEED = -1;
    public static final double FEEDER_SHOOT_SPEED = 1;

    /* Intake Constants */
    public static final double INTAKE_INTAKE_SPEED = 1;
    public static final double INTAKE_PURGE_SPEED = -1;

}