// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveTurnToLimelight;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.HoodHome;
import frc.robot.commands.HoodOverrideRestPosition;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ShooterAutoPrep;
import frc.robot.commands.ShooterAutoShoot;
import frc.robot.commands.ShooterSetSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1Ball extends SequentialCommandGroup {
    /** Creates a new Auto1Ball. */
    public Auto1Ball() {
        double cheese = 3685;

        addCommands(
            new DriveZeroGyro(270),
            new HoodOverrideRestPosition(true),
            new ParallelDeadlineGroup(
                new DriveFollowPath("1BallAuto", 3, 4),
                new HoodHome(1),
                new ShooterSetSpeed(0, cheese),
                new ShooterSetSpeed(1, cheese)
                ),
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new DriveTurnToLimelight(),
                new ShooterAutoPrep()
                ),
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new DriveTurnToLimelight(),
                new ShooterAutoPrep(),
                new ShootBalls(0, cheese),
                new ShootBalls(1, cheese)));
    }
}
