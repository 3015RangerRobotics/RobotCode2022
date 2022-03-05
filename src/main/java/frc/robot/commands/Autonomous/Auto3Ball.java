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
import frc.robot.subsystems.Intake.IntakeSolenoidPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto3Ball extends SequentialCommandGroup {
  /** Creates a new Auto3Ball. */
  public Auto3Ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double firstAngle = 24.5; //a guess!
    double firstSpeed = 3800; //a guess!
    double secondAngle = 24.5; //a guess!
    double secondSpeed = 3800; //a guess!
    addCommands(
      new DriveZeroGyro(72),
      new IntakeSetOverride(true),
      new IntakeSetPneumatic(IntakeSolenoidPosition.kDown),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(RobotContainer.intake[1]::getIntakeSensor).withTimeout(4),
        new IntakeBall(0),
        new IntakeBall(1),
        new HoodHome(1)),
      new ParallelDeadlineGroup(
        new DriveFollowPath("3BallAutopt1", 3, 4),
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
      new ShooterStop(0),
      new ShooterStop(1),
      new ParallelDeadlineGroup(
        new DriveFollowPath("3BallAutopt2", 3, 4),
        new IntakeBall(1)),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(RobotContainer.intake[1]::getIntakeSensor).withTimeout(1),
        new IntakeBall(1)),
      new ParallelDeadlineGroup(
        new DriveFollowPath("3BallAutopt3", 3, 4),
        new IntakeBall(0)),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(RobotContainer.intake[0]::getIntakeSensor).withTimeout(1),
        new IntakeBall(0)),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new DriveTurnToLimelight(),
          // new ShooterAutoPrep(),
        new HoodSetPosition(secondAngle),
        new ShooterSetSpeed(0, secondSpeed),
        new ShooterSetSpeed(1, secondSpeed)),
      new ParallelDeadlineGroup(
        new WaitCommand(0.75), 
        new ShootBalls(0, firstSpeed),
        new ShootBalls(1, firstSpeed)),
      new ShooterStop(0),
      new ShooterStop(1),
      new IntakeSetOverride(false));

  }
}
