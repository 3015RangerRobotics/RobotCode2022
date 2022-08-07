// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
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
    public static final int[] SWERVE_DRIVE_CHANNELS = { 11, 22, 50, 15 }; // { FR, FL, BL, BR }
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
    public static final int[] INTAKE_SOLENOID_FORWARD = {10, 14}; // { Left, Right }
    public static final int[] INTAKE_SOLENOID_REVERSE = {11, 13}; // { Left, Right }

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

    public static final double DRIVE_POS_ERROR_CONTROLLER_X_P = 1.1; // 1.1
    public static final double DRIVE_POS_ERROR_CONTROLLER_Y_P = 0.067;

    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_P = 2.4; // 8
    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_I = 0;
    public static final double DRIVE_AUTO_ROTATE_CONTROLLER_D = 0; // 0.65

    public static final double DRIVE_ROTATION_MIN_VELOCITY = 30;

    public static final int DRIVE_STATUS_FRAME_PERIOD = 20;

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
    public static final LookupTable SHOOTER_ZONE_TABLE = new LookupTable();
    public static final double SHOOTER_LL_ADJUST = 20; // 185

    public static final double SHOOTER_REST_SPEED = 1600;

    static {
        SHOOTER_LOOKUP_TABLE.put(40.4, 3200);
        SHOOTER_LOOKUP_TABLE.put(35.2, 3150);
        SHOOTER_LOOKUP_TABLE.put(45.0, 3150);
        SHOOTER_LOOKUP_TABLE.put(50.3, 3150);
        SHOOTER_LOOKUP_TABLE.put(55.0, 3150);
        SHOOTER_LOOKUP_TABLE.put(60.0, 3225);
        SHOOTER_LOOKUP_TABLE.put(65.0, 3380);
        SHOOTER_LOOKUP_TABLE.put(70.0, 3380);
        SHOOTER_LOOKUP_TABLE.put(75.0, 3500);
        SHOOTER_LOOKUP_TABLE.put(80.0, 3450);
        SHOOTER_LOOKUP_TABLE.put(85.0, 3465);
        SHOOTER_LOOKUP_TABLE.put(90.0, 3480);
        SHOOTER_LOOKUP_TABLE.put(95.0, 3525);
        SHOOTER_LOOKUP_TABLE.put(100.0, 3545);
        SHOOTER_LOOKUP_TABLE.put(105.0, 3595);
        SHOOTER_LOOKUP_TABLE.put(110.0, 3645);
        SHOOTER_LOOKUP_TABLE.put(115.0, 3685);
        SHOOTER_LOOKUP_TABLE.put(120.0, 3795);
        SHOOTER_LOOKUP_TABLE.put(125.0, 3825);
        SHOOTER_LOOKUP_TABLE.put(130.0, 3855);
        SHOOTER_LOOKUP_TABLE.put(135.0, 3905);
        SHOOTER_LOOKUP_TABLE.put(140.0, 3915);
        SHOOTER_LOOKUP_TABLE.put(145.0, 3915);
        SHOOTER_LOOKUP_TABLE.put(150.0, 3915);
        SHOOTER_LOOKUP_TABLE.put(155.0, 3945);
        SHOOTER_LOOKUP_TABLE.put(160.0, 4000);
        SHOOTER_LOOKUP_TABLE.put(165.0, 4070);
        SHOOTER_LOOKUP_TABLE.put(170.0, 4120);
        SHOOTER_LOOKUP_TABLE.put(175.0, 4120);
    }
    static {
        SHOOTER_ZONE_TABLE.put(65, 3685);
        SHOOTER_ZONE_TABLE.put(95, 3685);
        SHOOTER_ZONE_TABLE.put(115, 3935);
        SHOOTER_ZONE_TABLE.put(150, 3935);
    }

    /* Limelight Constants */
    public static final double LL_TARGET_HEIGHT = 2.64;// in meters
    public static final double LL_MOUNT_HEIGHT = (35.5) * (1.0 / 39.37008);
    public static final double LL_MOUNT_ANGLE = 25;
    public static final double LL_ROBOT_TO_TARGET = .183;
    public static final double LL_OFFSET = - (8.25 / 39.37008); // 8.25" in reality
    public static final double LL_BACK_OFFSET = 0.605; // add real
    public static final double LL_TOLERANCE = 1;
    public static final double LL_RIM_TO_FENDER = 0.1743;
    public static final double LL_GOAL_RADIUS = 1.355852;

    /* Climber Constants */
    public static final double CLIMBER_METERS_PER_PULSE = 0.0000056696;// Did: Falcon500 has 2048 count per revolution.
                                                                       // 12T on fal to 84T on same shaft w/ 12T sprock
                                                                       // to 15T sprock to Unknown diameter sprock
                                                                       // (rememner 16 Tooth sprock)
    public static final double CLIMBER_MAX_HEIGHT_METERS = .7229;
    public static final double CLIMBER_RELEASE_HEIGHT_METERS = CLIMBER_MAX_HEIGHT_METERS - .15;
    public static final double CLIMBER_HEIGHT_TOLERANCE = .05;

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
    public static final double HOOD_LL_ADJUST = 0;

    public static final LookupTable HOOD_LOOKUP_TABLE = new LookupTable();
    public static final LookupTable HOOD_ZONE_TABLE = new LookupTable();

    static {
        HOOD_LOOKUP_TABLE.put(35.2, 19.00);
        HOOD_LOOKUP_TABLE.put(40.4, 21.25);
        HOOD_LOOKUP_TABLE.put(45.0, 21.25);
        HOOD_LOOKUP_TABLE.put(50.3, 25.50);
        HOOD_LOOKUP_TABLE.put(55.0, 25.50);
        HOOD_LOOKUP_TABLE.put(60.0, 25.50);
        HOOD_LOOKUP_TABLE.put(65.0, 25.50);
        HOOD_LOOKUP_TABLE.put(70.0, 24.0);
        HOOD_LOOKUP_TABLE.put(75.0, 24.0);
        HOOD_LOOKUP_TABLE.put(80.0, 24.0);
        HOOD_LOOKUP_TABLE.put(85.0, 24.50);
        HOOD_LOOKUP_TABLE.put(90.0, 25.00);
        HOOD_LOOKUP_TABLE.put(95.0, 25.50);
        HOOD_LOOKUP_TABLE.put(100.0, 25.50);
        HOOD_LOOKUP_TABLE.put(105.0, 25.50);
        HOOD_LOOKUP_TABLE.put(110.0, 25.50);
        HOOD_LOOKUP_TABLE.put(115.0, 25.50);
        HOOD_LOOKUP_TABLE.put(120.0, 25.50);
        HOOD_LOOKUP_TABLE.put(125.0, 25.50);
        HOOD_LOOKUP_TABLE.put(130.0, 25.50);
        HOOD_LOOKUP_TABLE.put(135.0, 25.50);
        HOOD_LOOKUP_TABLE.put(140.0, 26.80);
        HOOD_LOOKUP_TABLE.put(145.0, 26.80);
        HOOD_LOOKUP_TABLE.put(150.0, 26.80);
        HOOD_LOOKUP_TABLE.put(155.0, 27.25);
        HOOD_LOOKUP_TABLE.put(160.0, 27.25);
        HOOD_LOOKUP_TABLE.put(165.0, 29.70);
        HOOD_LOOKUP_TABLE.put(170.0, 29.95);
        HOOD_LOOKUP_TABLE.put(175.0, 30.45);
    }

    static {
        HOOD_ZONE_TABLE.put(65, 25.2);
        HOOD_ZONE_TABLE.put(94.99, 25.2);
        HOOD_ZONE_TABLE.put(95, 27.2);
        HOOD_ZONE_TABLE.put(150, 27.2);

    }

    /* Feeder Constants */
    public static final double FEEDER_INTAKE_SPEED = 0.5;
    public static final double FEEDER_PURGE_SPEED = -0.4;
    public static final double FEEDER_SHOOT_SPEED = 0.8;
    public static final double FEEDER_TEST_SPEED = 0.5;

    /* Intake Constants */
    public static final double INTAKE_INTAKE_SPEED = 0.8;
    public static final double INTAKE_SLOW_SPEED = 0.5;
    public static final double INTAKE_PURGE_SPEED = -0.4;
    public static final double INTAKE_TEST_SPEED = 0.8;
    public static final I2C.Port[] I2C_PORTS = {I2C.Port.kOnboard, I2C.Port.kMXP}; // {left, right}
    public static final int[] COLOR_DRIFT = {92, 78};

    public static final int INTAKE_COLOR_THRESHOLD = 300;
}
