// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 2 Ball Autonomous */

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveSetBrakeMode;
import frc.robot.commands.DriveTurnToLimelight;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.HoodHome;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeSetOverride;
import frc.robot.commands.IntakeSetPneumatic;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.subsystems.Intake.IntakeSolenoidPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Ball extends SequentialCommandGroup {
  /** Creates a new Auto2Ball. */
  public Auto2Ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double firstSpeed = 3750;
    double firstAngle = 24.5;
    addCommands(
      new DriveSetBrakeMode(false),
      new DriveZeroGyro(317),
      new IntakeSetOverride(true),
      new IntakeSetPneumatic(IntakeSolenoidPosition.kDown),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(RobotContainer.intake[1]::getIntakeSensor).withTimeout(4),
        new HoodHome(),
        new IntakeBall(1)),
      new ParallelDeadlineGroup(
        new DriveFollowPath("2BallAutopt1", 3, 4), 
        new HoodSetPosition(firstAngle),
        new ShooterSetSpeed(0, firstSpeed),
        new ShooterSetSpeed(1, firstSpeed)),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new DriveTurnToLimelight(), 
        new IntakeBall(1),
        new HoodSetPosition(firstAngle)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.75), 
        new ShootBalls(0, 0),
        new ShootBalls(1, 0, 0.2, 0.2)),
      new ParallelDeadlineGroup(
        new DriveFollowPath("2BallAutopt2", 3, 4, false), 
        new IntakeBall(1)),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(RobotContainer.intake[1]::getIntakeSensor).withTimeout(1), 
        new DriveTurnToLimelight(), 
        new IntakeBall(1)),
      new ShooterSetSpeed(1, firstSpeed),
      new WaitCommand(0.8),
      new ShootBalls(1, 0).withTimeout(0.75),
      new DriveSetBrakeMode(true));
  }
}
