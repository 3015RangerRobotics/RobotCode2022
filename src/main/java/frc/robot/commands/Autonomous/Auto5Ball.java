// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 5 Ball Autonomous */

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAutoRotate;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveTurnToLimelight;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.HoodHome;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterAutoPrep;
import frc.robot.commands.ShooterAutoShoot;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto5Ball extends SequentialCommandGroup {
        /** Creates a new Auto5Ball. */
        public Auto5Ball() {
                double firstSpeed = 3800;
                double secondSpeed = 3800;
                double firstAngle = 24.5;
                double secondAngle = 24.5;
                addCommands(
                        new DriveZeroGyro(158),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt1", 3, 4),
                                new HoodHome(0.3),
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.8), 
                                new HoodHome(0.3),
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt2", 3, 4, false),
                                new IntakeBall(0),
                                new HoodHome(0.3),
                                new ShooterSetSpeed(0, firstSpeed),
                                new ShooterSetSpeed(1, firstSpeed)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.6),
                                new DriveTurnToLimelight(),
                                new ShooterSetSpeed(0, firstSpeed),
                                new ShooterSetSpeed(1, firstSpeed),
                                new HoodSetPosition(firstAngle)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.5), 
                                new ShootBalls(0, firstSpeed),
                                new ShootBalls(1, firstSpeed)),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt3", 2.5, 3, false), 
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.3),
                                new DriveTurnToLimelight(),
                                new ShooterSetSpeed(0, firstSpeed), 
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
                        new ParallelDeadlineGroup(
                                new WaitCommand(2), 
                                new IntakeBall(0)),
                        new ParallelDeadlineGroup(
                                new DriveFollowPath("5BallAutopt5", 3, 4, false), 
                                new ShooterSetSpeed(0, secondSpeed),
                                new HoodSetPosition(secondAngle)),
                        new ParallelDeadlineGroup(
                                new WaitCommand(0.8), 
                                new ShooterSetSpeed(0, secondSpeed),
                                new DriveTurnToLimelight()),
                        new ParallelDeadlineGroup(
                                new WaitCommand(1), 
                                new DriveTurnToLimelight(),
                                new ShootBalls(0, secondSpeed)),
                        new ShooterStop(0));
                System.out.println("===============================================\nAUTO HAS BEEN CREATED\n===============================================");
        }         
}