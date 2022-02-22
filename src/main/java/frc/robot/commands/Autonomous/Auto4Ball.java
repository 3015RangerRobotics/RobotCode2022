// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveFollowPathOld;
import frc.robot.commands.DriveTurnToLimelight;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterSetSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4Ball extends SequentialCommandGroup {
  /** Creates a new Auto4Ball. */
  public Auto4Ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double firstSpeed = 3500;
    double firstAngle = 30;
    addCommands(
        // Move to and intake first ball
        new ParallelDeadlineGroup(
            new DriveFollowPath("4BallAutopt1"),
            new IntakeBall(1), // Left Intake
            new HoodSetPosition(firstAngle)),
        // move to shooting pos while reving shooter
        new ParallelDeadlineGroup(
            new DriveFollowPath("4BallAutopt2"),
            new ShooterSetSpeed(0, firstSpeed),
            new ShooterSetSpeed(1, firstSpeed)),
        // ensures robot is on target
        new DriveTurnToLimelight(),
        // shoots the 2 balls
        new ParallelCommandGroup(
            new ShootBalls(1, firstSpeed),
            new ShootBalls(2, firstSpeed)),
        // Move to human player station to grab balls
        new ParallelDeadlineGroup(
            new DriveFollowPath("4BallAutopt3"),
            new IntakeBall(0)), // Right Intake
        // Drive to shooting position while reving shooter
        new ParallelDeadlineGroup(
            new DriveFollowPath("4BallAutopt4"),
            new ShooterSetSpeed(0, firstSpeed),
            new ShooterSetSpeed(1, firstSpeed)),
        // ensures robot is on target
        new DriveTurnToLimelight(),
        // shoot the balls
        new ParallelCommandGroup(
            new ShootBalls(1, firstSpeed),
            new ShootBalls(0, firstSpeed)));
  }
}
