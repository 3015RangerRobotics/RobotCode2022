// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeSetColorOverride;
import frc.robot.commands.IntakeSetPneumatic;
import frc.robot.commands.PurgeBall;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2BallSteal extends SequentialCommandGroup {
    /** Creates a new Auto2BallSteal. */
    public Auto2BallSteal() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new Auto2Ball(),
                new IntakeSetColorOverride(0, true),
                new ParallelDeadlineGroup(
                        new DriveFollowPath("2BallAutoStealPt1", 3, 4),
                        new IntakeSetPneumatic(0, true),
                        new IntakeBall(0)),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(RobotContainer.intakeFeeder[0]::getIntakeSensor).withTimeout(1),
                        new IntakeBall(0)),
                new IntakeSetPneumatic(0, false),
                new ParallelCommandGroup(
                        new DriveFollowPath("2BallAutoStealPt2", 3, 4),
                        new IntakeBall(0).withTimeout(0.2)),
                new WaitCommand(0.2),
                new IntakeSetColorOverride(0, false),
                new PurgeBall(0, -0.275));
    }
}
