// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveTurnToLimelight;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeSetOverride;
import frc.robot.commands.IntakeSetPneumatic;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Intake.IntakeSolenoidPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4Ball2 extends SequentialCommandGroup {
  /** Creates a new Auto4Ball2. */
  public Auto4Ball2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double speed = 3800;
    double angle = 24.5;
    addCommands(
      new DriveZeroGyro(315),
      new IntakeSetOverride(0, true),
      new IntakeSetPneumatic(0, true),
      new IntakeSetOverride(1, true),
      new IntakeSetPneumatic(1, true),
      new ParallelDeadlineGroup(
        new DriveFollowPath("4BallAuto2pt1", 3, 4),
        new IntakeBall(1),
        new HoodSetPosition(angle),
        new ShooterSetSpeed(0, speed),
        new ShooterSetSpeed(1, speed)),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(RobotContainer.intakeFeeder[1]::getIntakeSensor).withTimeout(1),
        new IntakeBall(1)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.6),
        new DriveTurnToLimelight(),
        new HoodSetPosition(angle),
        new ShooterSetSpeed(0, speed),
        new ShooterSetSpeed(1, speed)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.75), 
        new ShootBalls(0, speed),
        new ShootBalls(1, speed)),
      new ShooterStop(0),
      new ShooterStop(1),
      new ParallelDeadlineGroup(
        new DriveFollowPath("4BallAuto2pt2", 3, 4),
        new IntakeBall(1)),
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new IntakeBall(1)),
      new ParallelDeadlineGroup(
        new DriveFollowPath("4BallAuto2pt3", 3, 4),
        new HoodSetPosition(angle),
        new ShooterSetSpeed(0, speed),
        new ShooterSetSpeed(1, speed)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.6),
        new DriveTurnToLimelight(),
        new HoodSetPosition(angle),
        new ShooterSetSpeed(0, speed),
        new ShooterSetSpeed(1, speed)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.75), 
        new ShootBalls(0, speed),
        new ShootBalls(1, speed)),
      new ShooterStop(0),
      new ShooterStop(1),
      new IntakeSetOverride(0, false)
      
    
    );
  }
}
