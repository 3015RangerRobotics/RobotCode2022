// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 5 Ball Autonomous */

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveSetBrakeMode;
import frc.robot.commands.DriveSetModuleAngles;
import frc.robot.commands.DriveTurnToLimelight;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.HoodHome;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeSetOverride;
import frc.robot.commands.IntakeSetPneumatic;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto5Ball extends SequentialCommandGroup {
        /** Creates a new Auto5Ball. */
        public Auto5Ball() {
                double firstSpeed = 3785;
                double secondSpeed = 3785;
                double firstAngle = 24.5;
                double secondAngle = 24.5;
                addCommands(
                        // new DriveSetBrakeMode(true),

                        new DriveZeroGyro(182),
                        new IntakeSetOverride(0, true),
                        new IntakeSetPneumatic(0, true),
                        // new DriveSetModuleAngles(90),
                        //new CompressorSetEnabled(true),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt1", 3, 4),
                                new HoodHome(1),
                                new ShooterSetSpeed(0, firstSpeed),
                                new ShooterSetSpeed(1, firstSpeed),
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(RobotContainer.intakeFeeder[0]::getIntakeSensor).withTimeout(0.8),
                                new HoodHome(1),
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt2", 3, 4, false),
                                new IntakeBall(0),
                                // new HoodHome(1),
                                new HoodSetPosition(firstAngle),
                                new ShooterSetSpeed(0, firstSpeed),
                                new ShooterSetSpeed(1, firstSpeed)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.6),
                                new DriveTurnToLimelight(),
                                // new ShooterAutoPrep(),
                                new HoodSetPosition(firstAngle),
                                new ShooterSetSpeed(0, firstSpeed),
                                new ShooterSetSpeed(1, firstSpeed)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.75), 
                                new ShootBalls(0, firstSpeed),
                                new ShootBalls(1, firstSpeed)),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt3", 3, 4, false), 
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.3),
                                new DriveTurnToLimelight(),
                                // new ShooterAutoPrep(), 
                                new ShooterSetSpeed(0, firstSpeed),
                                new ShooterSetSpeed(1, firstSpeed),
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.5), 
                                new DriveTurnToLimelight(),
                                new ShootBalls(0, firstSpeed)),
                        new ShooterStop(0),
                        new ShooterStop(1),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt4", 3, 4, false), 
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup( // 1.5
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(RobotContainer.intakeFeeder[0]::getFeederDetector),
                                        new WaitUntilCommand(RobotContainer.intakeFeeder[0]::getIntakeSensor)
                                ).withTimeout(1.5),
                                new IntakeBall(0)),
                        //new IntakeSetPneumatic(Intake.IntakeSolenoidPosition.kUp),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt5", 4, 4.5, false), 
                                new ShooterSetSpeed(0, secondSpeed),
                                new HoodSetPosition(secondAngle),
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.8), 
                                new ShooterSetSpeed(0, secondSpeed),
                                // new ShooterAutoPrep(),
                                new DriveTurnToLimelight()),
                        //new CompressorSetEnabled(true),
                        new ParallelDeadlineGroup(
                                new WaitCommand(1), 
                                new DriveTurnToLimelight(),
                                new ShootBalls(0, secondSpeed)),
                        new ShooterStop(0),
                        new ShooterStop(1),
                        new IntakeSetOverride(0, false));
        }         
}