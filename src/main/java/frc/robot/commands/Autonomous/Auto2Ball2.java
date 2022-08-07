// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveAutoRotate;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.HoodHome;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeSetColorOverride;
import frc.robot.commands.IntakeSetOverride;
import frc.robot.commands.IntakeSetPneumatic;
import frc.robot.commands.PurgeBall;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterAutoPrep;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Ball2 extends SequentialCommandGroup {
  /** Creates a new Auto2Ball2. */
  public Auto2Ball2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveZeroGyro(225),
        new IntakeSetOverride(0, true),
        new IntakeSetPneumatic(0, false),
        new IntakeSetOverride(1, true),
        new IntakeSetPneumatic(1, true),
        new IntakeSetColorOverride(0, true),
        new IntakeSetColorOverride(1, true),
        new ParallelDeadlineGroup(
            new DriveFollowPath("2BallAuto2pt1", 3, 4, true), 
            new HoodHome(),
            new IntakeBall(1)),
        new ParallelDeadlineGroup(
            // new WaitUntilCommand(RobotContainer.intakeFeeder[1]::getIntakeSensor).withTimeout(1),
            new WaitCommand(1),
            new HoodHome(),
            new IntakeBall(1)),
        new ParallelDeadlineGroup(
            new WaitCommand(1), 
            new HoodHome(),
            new DriveTurnToAngle(45)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(1.5),
            new DriveAutoRotate(),
            new ShooterAutoPrep()),
        new ParallelDeadlineGroup(
            new WaitCommand(0.8),
            new ShootBalls(0, 0),
            new ShootBalls(1, 0)),
        new ParallelDeadlineGroup(
            new DriveFollowPath("2BallAuto2pt2", 3, 4, false), 
            new IntakeBall(0)),
        new ParallelDeadlineGroup(
            new WaitUntilCommand(RobotContainer.intakeFeeder[0]::getIntakeSensor).withTimeout(1), 
            new IntakeBall(0)),
        new DriveFollowPath("2BallAuto2pt3", 3, 4, false),
        new PurgeBall(0, -0.275)
    );
  }
}
