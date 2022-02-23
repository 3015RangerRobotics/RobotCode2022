// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import lib.LookupTable;

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

    /* Hood */
    public static final int HOOD_MOTOR = 28;

    /* Compressor */
    public static final int PNEUMATIC_HUB = 1;

    /* Solenoid ID's */
    /* Climber */
    public static final int CLIMBER_SOLENOID_UP = 0;
    public static final int CLIMBER_SOLENOID_DOWN = 1;

    /* Digital IO ID's */
    /* climber */
    public static final int CLIMBER_BOTTOM_SWITCH = 5;
    public static final int HOOD_SWITCH_CHANNEL = 4;
    public static final int[] FEEDER_BALL_DETECTORS = { 7, 6 }; // { left, right }
    public static final int[] INTAKE_BALL_DETECTORS = { 9, 8 }; // { left, right }
    public static final int ENCODER_INPUT_A = 1;
    public static final int ENCODER_INPUT_B = 2;

    /* -------------- CONSTANTS -------------- */
    /* Drive Constants */

    public static final double SWERVE_MAX_VELOCITY_METERS = 4.8; // 15.9 ft/s
    public static final double SWERVE_MAX_ACCEL_METERS = 5;
    public static final double SWERVE_METERS_PER_PULSE = 0.00002226;
    public static final double SWERVE_DEGREES_PER_PULSE = 360.0 / 4096.0;
    public static final double DRIVE_DEADZONE = 0.15;

    public static final double DRIVE_MAX_ANGULAR_ACCEL = 600;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 300;

    public static final double SWERVE_CENTER_DISTANCE = .345;

    public static final double SWERVE_DRIVE_P = -1000;
    public static final double SWERVE_DRIVE_I = 0;
    public static final double SWERVE_DRIVE_D = -25;
    public static final double SWERVE_DRIVE_F = 0;

    public static final double[] SWERVE_ROTATION_P = { 8, 8, 8, 8 };
    public static final double[] SWERVE_ROTATION_I = { 0.2, 0.2, 0.2, 0.2 };
    public static final double[] SWERVE_ROTATION_D = { 2500, 2500, 2500, 2500 };
    public static final double SWERVE_ROTATION_I_ZONE = 10 / SWERVE_DEGREES_PER_PULSE;
    public static final double SWERVE_ROTATION_F = 0;

    public static final double DRIVE_ROTATION_CONTROLLER_P = 8;
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0.2;
    public static final double DRIVE_ROTATION_CONTROLLER_F = 0;

    public static final double DRIVE_POS_ERROR_CONTROLLER_P = 0.1;
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_F = 0;

    public static final double DRIVE_HEADING_ERROR_CONTROLLER_P = 0.03;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_D = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_F = 0;

    public static final double DRIVE_ROTATION_MIN_VELOCITY = 0; // place holder

    public static final Translation2d DRIVE_GOAL_POSITION = new Translation2d(8.2296, 4.1148);

    /* Shooter Constants */
    public static final double SHOOTER_MOTOR_SPEED = 10;

    public static final double SHOOTER_P = 0.5;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 5;
    public static final double SHOOTER_F = 0.0468;

    public static final double SHOOTER_SHOOT_P = 1;
    public static final double SHOOTER_SHOOT_I = 0;
    public static final double SHOOTER_SHOOT_D = 0;
    public static final double SHOOTER_SHOOT_F = 0;

    public static final double SHOOTER_PULSES_PER_ROTATION = 2048;

    public static final double SHOOTER_TOLERANCE = .05; // ??
    public static final LookupTable SHOOTER_LOOKUP_TABLE = new LookupTable();

    static {
        SHOOTER_LOOKUP_TABLE.put(2.5, 3000.0);
        SHOOTER_LOOKUP_TABLE.put(11.0, 3200.0);
        SHOOTER_LOOKUP_TABLE.put(27.0, 3200.0);
        SHOOTER_LOOKUP_TABLE.put(45.0, 3300.0);
        SHOOTER_LOOKUP_TABLE.put(62.0, 3400.0);
        SHOOTER_LOOKUP_TABLE.put(77.0, 3400.0);
        SHOOTER_LOOKUP_TABLE.put(96.0, 3550.0);
        SHOOTER_LOOKUP_TABLE.put(110.0, 3650.0);
        SHOOTER_LOOKUP_TABLE.put(124.0, 3750.0);
        SHOOTER_LOOKUP_TABLE.put(136.0, 4000.0);
        SHOOTER_LOOKUP_TABLE.put(158.0, 4100.0);
        SHOOTER_LOOKUP_TABLE.put(178.0, 4200.0);
        SHOOTER_LOOKUP_TABLE.put(192.0, 4500.0);
    }

    /* Limelight Constants */
    public static final double LL_TARGET_HEIGHT = 2.64;// in meters
    public static final double LL_MOUNT_HEIGHT = 0.8001; // 31.5 inches
    public static final double LL_MOUNT_ANGLE = 30;
    public static final double LL_ROBOT_TO_TARGET = .183;
    public static final double LL_OFFSET = -0.178; // add real
    public static final double LL_BACK_OFFSET = 0.605; // add real
    public static final double LL_TOLERANCE = 1;
    public static final double LL_GOAL_RADIUS = 1.355852;

    /* Climber Constants */
    public static final double CLIMBER_METERS_PER_PULSE = 0.0000056696;// Did: Falcon500 has 2048 count per revolution.
                                                                       // 12T on fal to 84T on same shaft w/ 12T sprock
                                                                       // to 15T sprock to Unknown diameter sprock
                                                                       // (rememner 16 Tooth sprock)
    public static final double CLIMBER_MAX_HEIGHT_METERS = .72;
    public static final double CLIMBER_RELEASE_HEIGHT_METERS = CLIMBER_MAX_HEIGHT_METERS - .15;
    public static final double CLIMBER_HEIGHT_TOLERANCE = .005;

    public static final double CLIMBER_RAISE_P = 1;
    public static final double CLIMBER_RAISE_I = 0;
    public static final double CLIMBER_RAISE_D = 0;
    public static final double CLIMBER_RAISE_F = 0;

    public static final double CLIMBER_LOWER_P = 1;
    public static final double CLIMBER_LOWER_I = 0;
    public static final double CLIMBER_LOWER_D = 0;
    public static final double CLIMBER_LOWER_F = 3;

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

    public static final double HOOD_DEGREES_PER_ROTATION = (1 / 933.333) * 360;
    public static final double HOOD_MAX_ANGLE = 32;

    public static final double HOOD_TOLERANCE = .5;
    public static final LookupTable HOOD_POSITION_TABLE = new LookupTable();
    static {
        HOOD_POSITION_TABLE.put(2.5, 4.2);
        HOOD_POSITION_TABLE.put(11.0, 4.75);
        HOOD_POSITION_TABLE.put(27.0, 8.0);
        HOOD_POSITION_TABLE.put(45.0, 12.5);
        HOOD_POSITION_TABLE.put(62.0, 14.8);
        HOOD_POSITION_TABLE.put(77.0, 17.0);
        HOOD_POSITION_TABLE.put(96.0, 18.1);
        HOOD_POSITION_TABLE.put(110.0, 18.1);
        HOOD_POSITION_TABLE.put(124.0, 19.1);
        HOOD_POSITION_TABLE.put(136.0, 19.8);
        HOOD_POSITION_TABLE.put(158.0, 22.2);
        HOOD_POSITION_TABLE.put(178.0, 23.8);
        HOOD_POSITION_TABLE.put(192.0, 27.0);
    }

    /* Feeder Constants */
    public static final double FEEDER_INTAKE_SPEED = 0.5;
    public static final double FEEDER_PURGE_SPEED = -0.4;
    public static final double FEEDER_SHOOT_SPEED = 0.8;

    /* Intake Constants */
    public static final double INTAKE_INTAKE_SPEED = 0.8;
    public static final double INTAKE_PURGE_SPEED = -0.4;
}