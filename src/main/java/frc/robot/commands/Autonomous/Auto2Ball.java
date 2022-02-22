// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveFollowPathOld;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterSetSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Ball extends SequentialCommandGroup {
  /** Creates a new Auto2Ball. */
  public Auto2Ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double firstSpeed = 3500;
    double firstAngle = 30;
    addCommands(
                  // Move to and intake first ball while revving shooters
                  new ParallelDeadlineGroup(
                     new DriveFollowPath("2BallAutopt1"),
                     new IntakeBall(1), // Left Intake
                     new ShooterSetSpeed(0, firstSpeed),
                     new ShooterSetSpeed(1, firstSpeed),
                     new HoodSetPosition(firstAngle)),
                  // Move to shooting position and shoot 2 balls
                  new ParallelDeadlineGroup(
                     new DriveFollowPath("2BallAutopt2"),
                     new ShootBalls(0, firstSpeed),
                     new ShootBalls(1, firstSpeed)));
  }
}
