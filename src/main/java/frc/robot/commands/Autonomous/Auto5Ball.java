// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterSetSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto5Ball extends SequentialCommandGroup {
    /** Creates a new Auto5Ball. */
    public Auto5Ball() {
        double firstSpeed = 5000;
        double secondSpeed = 5000;
        double firstAngle = 30;
        double secondAngle = 30;
        addCommands(
                // Move to and intake first ball while revving shooters
                new ParallelDeadlineGroup(
                        new DriveFollowPath("firstBall5"),
                        new IntakeBall(0), // Left Intake
                        new ShooterSetSpeed(0, firstSpeed),
                        new ShooterSetSpeed(1, firstSpeed),
                        new HoodSetPosition(firstAngle)),
                // Move just shy of second ball while keeping shooter running
                new ParallelDeadlineGroup(
                        new DriveFollowPath("secondBall5"),
                        new ShooterSetSpeed(0, firstSpeed),
                        new ShooterSetSpeed(1, firstSpeed)),
                // Once at position, shoot balls while moving slightly to pickup second ball
                new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new DriveFollowPath("secondBallPickup5")),
                        new ShootBalls(0, firstSpeed),
                        new ShootBalls(1, firstSpeed)),
                // Move to station for 4th and 5th balls
                new ParallelDeadlineGroup(
                        new DriveFollowPath("moveToStation5"),
                        new IntakeBall(0)),
                // At station wait for ball from human player (may need to increase wait time)
                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new ShooterSetSpeed(0, secondSpeed)),
                // Move to final shooting position while revving shooter and setting hood
                new ParallelDeadlineGroup(
                        new DriveFollowPath("finalMove5"),
                        new ShooterSetSpeed(0, secondSpeed),
                        new HoodSetPosition(secondAngle)),
                // Fire final two balls, which will both be in the left side
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new ShootBalls(0, secondSpeed, 0.75)));
    }
}
