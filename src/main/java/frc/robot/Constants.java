// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* CAN IDs */
        /* Swerve Modules */
        public static final int SWERVE_DRIVE_CHANNEL_FL = 0;
        public static final int SWERVE_DRIVE_CHANNEL_FR = 0;
        public static final int SWERVE_DRIVE_CHANNEL_BL = 0;
        public static final int SWERVE_DRIVE_CHANNEL_BR = 0;

        public static final int SWERVE_ROTATION_CHANNEL_FL = 0;
        public static final int SWERVE_ROTATION_CHANNEL_FR = 0;
        public static final int SWERVE_ROTATION_CHANNEL_BL = 0;
        public static final int SWERVE_ROTATION_CHANNEL_BR = 0;

        public static final int DRIVE_PIGEON_CHANNEL = 0;

    /* Drive Constants */
        public static final double SWERVE_DRIVE_P = 0;
        public static final double SWERVE_DRIVE_I = 0;
        public static final double SWERVE_DRIVE_D = 0;
        public static final double SWERVE_DRIVE_F = 0;

    public static final double SWERVE_ROTATION_P = 0;
    public static final double SWERVE_ROTATION_I = 0;
    public static final double SWERVE_ROTATION_D = 0;
    public static final double SWERVE_ROTATION_F = 0;

    public static final double SWERVE_ROTATION_I_ZONE = 0;

    public static final double SWERVE_MAX_VELOCITY = 0;
    public static final double SWERVE_METERS_PER_PULSE = 0;
    public static final double SWERVE_DEGREES_PER_PULSE = 0;

    public static final double SWERVE_CENTER_DISTANCE = 0;

    //Shooter Constants
    public static final double SHOOTER_MOTOR_SPEED = 0;

    //Hood Constants
    public static final double HOOD_MOTOR_CONTROLLER = 0;

    //Feeder Constants
    public static final double FEEDER_TOP_MOTOR1 = 0;
    public static final double FEEDER_BOTTOM_MOTOR1 = 0;
    public static final double FEEDER_TOP_MOTOR2 = 0;
    public static final double FEEDER_BOTTOM_MOTOR2 = 0;

    //Climber Constants
    public static final double CLIMBER_MOTOR = 0;
}
