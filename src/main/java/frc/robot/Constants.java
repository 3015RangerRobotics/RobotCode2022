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
    public static final int[] SWERVE_DRIVE_CHANNELS = { 11, 13, 15, 17 }; // { FR, FL, BL, BR }
    public static final int[] SWERVE_ROTATION_CHANNELS = { 12, 14, 16, 18 }; // { FR, FL, BL, BR }
    public static final int DRIVE_PIGEON_CHANNEL = 1;

    /* Intake */
    public static final int[] INTAKE_MOTORS = { 24, 26 }; // { left, right }

    /* Feeder */
    public static final int[] FEEDER_TOP_MOTORS = { 25, 27 }; // { left, right }

    /* Shooter */
    public static final int[] SHOOTER_MOTORS = { 22, 23 }; // { left, right }

    /* Climber */
    public static final int CLIMBER_MOTOR = 31;

    /* REV ID's */
    /* Hood */
    public static final int HOOD_MOTOR = 11; // { left and right }

    /* Solenoid ID's */
    /* Climber */
    public static final int CLIMBER_SOLENOID_UP = 1; // TEMP
    public static final int CLIMBER_SOLENOID_DOWN = 2; // TEMP

    /* Digital IO ID's */
    /* climber */
    public static final int CLIMBER_BOTTOM_SWITCH = 0;
    public static final int CLIMBER_BEAMBREAK_SENSOR = 1;
    public static final int[] FEEDER_BALL_DETECTORS = { 2, 3 }; // { left, right }
    public static final int[] INTAKE_BALL_DETECTORS = { 4, 5 }; // { left, right }
    public static final int ENCODER_INPUT_A = 6;
    public static final int ENCODER_INPUT_B = 7;

    /* -------------- CONSTANTS -------------- */
    /* Drive Constants */

    public static final double SWERVE_MAX_VELOCITY_METERS = 4.8; // 15.9 ft/s
    public static final double SWERVE_MAX_ACCEL_METERS = 5;
    public static final double SWERVE_METERS_PER_PULSE = 0.00002226;
    public static final double SWERVE_DEGREES_PER_PULSE = 360.0 / 4096.0;
    public static final double DRIVE_DEADZONE = 0.15;

    public static final double DRIVE_MAX_ANGULAR_ACCEL = 0;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 300;

    public static final double SWERVE_CENTER_DISTANCE = .345;

    public static final double SWERVE_DRIVE_P = -1000;
    public static final double SWERVE_DRIVE_I = 0;
    public static final double SWERVE_DRIVE_D = -25;
    public static final double SWERVE_DRIVE_F = 0;

    public static final double SWERVE_ROTATION_P = 16;
    public static final double SWERVE_ROTATION_I = .04;
    public static final double SWERVE_ROTATION_I_ZONE = 10 / SWERVE_DEGREES_PER_PULSE;
    public static final double SWERVE_ROTATION_D = 1600;
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

    /* Shooter Constants */
    public static final double SHOOTER_MOTOR_SPEED = 10;

    public static final double SHOOTER_P = 1;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_F = 0;

    public static final double SHOOTER_SHOOT_P = 1;
    public static final double SHOOTER_SHOOT_I = 0;
    public static final double SHOOTER_SHOOT_D = 0;
    public static final double SHOOTER_SHOOT_F = 0;

    public static final double SHOOTER_PULSES_PER_ROTATION = 2048;

    /* Limelight Constants */
    public static final double LL_TARGET_HEIGHT = 2.64;// in meters
    public static final double LL_MOUNT_HEIGHT = 0;
    public static final double LL_MOUNT_ANGLE = 55;

    /* Climber Constants */
    public static final double CLIMBER_METERS_PER_PULSE = 0.0000056696;// Did: Falcon500 has 2048 count per revolution.
                                                                       // 12T on fal to 84T on same shaft w/ 12T sprock
                                                                       // to 15T sprock to Unknown diameter sprock
                                                                       // (rememner 16 Tooth sprock)
    public static final double CLIMBER_MAX_HEIGHT_METERS = .7266;
    public static final double CLIMBER_HEIGHT_TOLERANCE = .005;

    public static final double CLIMBER_RAISE_P = 1;
    public static final double CLIMBER_RAISE_I = 0;
    public static final double CLIMBER_RAISE_D = 0;
    public static final double CLIMBER_RAISE_F = 0;

    public static final double CLIMBER_LOWER_P = 1;
    public static final double CLIMBER_LOWER_I = 0;
    public static final double CLIMBER_LOWER_D = 0;
    public static final double CLIMBER_LOWER_F = 0;

    public static final double CLIMBER_ARM_DEGREES_PER_PULSE = 0.17578;// found! TODO:this is a RevThroughBoreEncoder
                                                                       // 2048 pulses per rev, Find this value
    public static final double CLIMBER_FLOPPY_BAR_TWO_TO_THREE = 0;// TODO: Do math to find me
    public static final double CLIMBER_FLOPPY_BAR_THREE_TO_FOUR = 0;// TODO: Do math to find me
    public static final double CLIMBER_FLOPPY_POSITION_TOLERANCE = 4;
    public static final double CLIMBER_FLOPPY_SPEED_TOLERANCE = 7;
    public static final double CLIMBER_FLOPPY_ARM_OFFSET = 22;// Looked up on 2/8/22 using actual robot

    /* Hood Constants */
    public static final double HOOD_CONTROLLER_P = 1;
    public static final double HOOD_CONTROLLER_I = 0;
    public static final double HOOD_CONTROLLER_D = 0;
    public static final double HOOD_CONTROLLER_F = 0;
    public static final double HOOD_DEGREES_PER_ROTATION = 1;// finding! 0.00856? TODO: Get the gear ratios- 18:36 ?,
                                                             // NEO550 have 42 counts per rotation.
    public static final double HOOD_MAX_ANGLE = 0;

    /* Feeder Constants */
    public static final double FEEDER_INTAKE_SPEED = 1;
    public static final double FEEDER_PURGE_SPEED = -1;
    public static final double FEEDER_SHOOT_SPEED = 1;

    /* Intake Constants */
    public static final double INTAKE_INTAKE_SPEED = 1;
    public static final double INTAKE_PURGE_SPEED = -1;

}