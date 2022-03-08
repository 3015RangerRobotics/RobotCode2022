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
    public static final int[] SWERVE_DRIVE_CHANNELS = { 11, 22, 15, 17 }; // { FR, FL, BL, BR }
    public static final int[] SWERVE_ROTATION_CHANNELS = { 12, 14, 16, 18 }; // { FR, FL, BL, BR }
    public static final int DRIVE_PIGEON_CHANNEL = 1;

    /* Intake */
    public static final int[] INTAKE_MOTORS = { 24, 26 }; // { left, right }

    /* Feeder */
    public static final int[] FEEDER_TOP_MOTORS = { 25, 27 }; // { left, right }

    /* Shooter */
    public static final int[] SHOOTER_MOTORS = { 34, 23 }; // { left, right }

    /* Climber */
    public static final int CLIMBER_MOTOR = 31;

    /* Hood */
    public static final int HOOD_MOTOR = 28;

    /* Compressor */
    public static final int PNEUMATIC_HUB = 1;

    /* Solenoid ID's */
    /* Climber */
    public static final int CLIMBER_SOLENOID_FORWARD = 15;
    public static final int CLIMBER_SOLENOID_REVERSE = 12;

    /* Intake */
    public static final int INTAKE_SOLENOID_FORWARD = 14;
    public static final int INTAKE_SOLENOID_REVERSE = 13;

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

    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_P = 2.4;
    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_I = 0;
    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_D = 0;

    public static final double DRIVE_ROTATION_MIN_VELOCITY = 0; // place holder

    public static final Translation2d DRIVE_GOAL_POSITION = new Translation2d(8.2296, 4.1148);

    /* Shooter Constants */
    public static final double SHOOTER_MOTOR_SPEED = 10;

    public static final double SHOOTER_P = 0.1;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_F = 0.04485;

    public static final double SHOOTER_SHOOT_P = 1;
    public static final double SHOOTER_SHOOT_I = 0;
    public static final double SHOOTER_SHOOT_D = 0;
    public static final double SHOOTER_SHOOT_F = 0;

    public static final double SHOOTER_PULSES_PER_ROTATION = 2048;

    public static final double SHOOTER_TOLERANCE = .0065;
    public static final LookupTable SHOOTER_LOOKUP_TABLE = new LookupTable();
    public static final double SHOOTER_LL_ADJUST = 75;

    public static final double SHOOTER_REST_SPEED = 1600;

    static {
        SHOOTER_LOOKUP_TABLE.put(35, 3300);
        SHOOTER_LOOKUP_TABLE.put(40, 3350);
        SHOOTER_LOOKUP_TABLE.put(50, 3400);
        SHOOTER_LOOKUP_TABLE.put(60, 3400);
        SHOOTER_LOOKUP_TABLE.put(73, 3425);
        SHOOTER_LOOKUP_TABLE.put(83, 3450);
        SHOOTER_LOOKUP_TABLE.put(90, 3450);
        SHOOTER_LOOKUP_TABLE.put(100, 3475);
        SHOOTER_LOOKUP_TABLE.put(110, 3550);
        SHOOTER_LOOKUP_TABLE.put(120, 3675);
        SHOOTER_LOOKUP_TABLE.put(130, 3725);
        SHOOTER_LOOKUP_TABLE.put(140, 3800);
        SHOOTER_LOOKUP_TABLE.put(150, 3900);
    }

    /* Limelight Constants */
    public static final double LL_TARGET_HEIGHT = 2.64;// in meters
    public static final double LL_MOUNT_HEIGHT = 0.8001; // 31.5 inches
    public static final double LL_MOUNT_ANGLE = 25;
    public static final double LL_ROBOT_TO_TARGET = .183;
    public static final double LL_OFFSET = - (8.25 / 39.37008); // 8.25"
    public static final double LL_BACK_OFFSET = 0.605; // add real
    public static final double LL_TOLERANCE = 1;
    public static final double LL_RIM_TO_FENDER = 0.1743;
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
    public static final double HOOD_CONTROLLER_P = 10;
    public static final double HOOD_CONTROLLER_I = 0;
    public static final double HOOD_CONTROLLER_D = 0;
    public static final double HOOD_CONTROLLER_F = 0;

    public static final double HOOD_DEGREES_PER_PULSE = ((1 / 23.3333) * 360.0) / 4096.0; // 1:23.333 ratio * 360 deg / 1024 pulses per rotation
    public static final double HOOD_MAX_ANGLE = 32;

    public static final double HOOD_TOLERANCE = .5;
    public static final double HOOD_REST_POSITION = 22;

    public static final LookupTable HOOD_POSITION_TABLE = new LookupTable();
    static {
        HOOD_POSITION_TABLE.put(35, 12.5);
        HOOD_POSITION_TABLE.put(40, 12.5);
        HOOD_POSITION_TABLE.put(50, 14.5);
        HOOD_POSITION_TABLE.put(60, 17.5);
        HOOD_POSITION_TABLE.put(73, 19.0);
        HOOD_POSITION_TABLE.put(83, 20.0);
        HOOD_POSITION_TABLE.put(90, 21.0);
        HOOD_POSITION_TABLE.put(100, 22.5);
        HOOD_POSITION_TABLE.put(110, 24.0);
        HOOD_POSITION_TABLE.put(120, 24.0);
        HOOD_POSITION_TABLE.put(130, 24.0);
        HOOD_POSITION_TABLE.put(140, 24.0);
        HOOD_POSITION_TABLE.put(150, 24.0);
    }

    /* Feeder Constants */
    public static final double FEEDER_INTAKE_SPEED = 0.5;
    public static final double FEEDER_PURGE_SPEED = -0.4;
    public static final double FEEDER_SHOOT_SPEED = 0.8;
    public static final double FEEDER_TEST_SPEED = 0.4;

    /* Intake Constants */
    public static final double INTAKE_INTAKE_SPEED = 0.8;
    public static final double INTAKE_PURGE_SPEED = -0.4;
    public static final double INTAKE_TEST_SPEED = 0.4;
}