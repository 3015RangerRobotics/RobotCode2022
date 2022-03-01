// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 4 Ball Autonomous */

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.DriveTurnToLimelight;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.HoodHome;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeSetPneumatic;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeSolenoidPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4Ball extends SequentialCommandGroup {
  /** Creates a new Auto4Ball. */
  public Auto4Ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double firstSpeed = 3800;
    double firstAngle = 24.5;
    addCommands(
        new DriveZeroGyro(115),
        new ParallelDeadlineGroup(
            new DriveFollowPath("4BallAutopt1", 3, 4), 
            new HoodHome(0.3),
            new IntakeBall(0)),
        new ParallelDeadlineGroup(
            new WaitCommand(0.5), 
            new HoodHome(0.5),
            new IntakeBall(0)),
        new ParallelDeadlineGroup(
            new WaitCommand(1.2),
            new DriveTurnToAngle(-30),
            new HoodSetPosition(firstAngle),
            new ShooterSetSpeed(0, firstSpeed),
            new ShooterSetSpeed(1, firstSpeed)),
        new ParallelDeadlineGroup(
            new WaitCommand(0.8), 
            new DriveTurnToLimelight()),
        new ParallelDeadlineGroup(
            new WaitCommand(0.5),
            new ShootBalls(0, 0),
            new ShootBalls(1, 0)),
        new ShooterStop(0),
        new ShooterStop(1),
        new ParallelDeadlineGroup(
            new DriveFollowPath("4BallAutopt2", 3, 4, false), 
            new IntakeBall(0)),
        new ParallelDeadlineGroup(
            new WaitCommand(2), 
            new IntakeBall(0)),
        new IntakeSetPneumatic(IntakeSolenoidPosition.kUp),
        new ParallelDeadlineGroup(
            new DriveFollowPath("4BallAutopt3", 3, 4, false), 
            new HoodSetPosition(firstAngle),
            new ShooterSetSpeed(0, firstSpeed)),
        new ParallelDeadlineGroup(
            new WaitCommand(1.2), 
            new DriveTurnToLimelight()),
        new ParallelDeadlineGroup(
            new WaitCommand(1), 
            new ShootBalls(0, 0, 0.4)),
        new ShooterStop(0)
    );
  }
}
