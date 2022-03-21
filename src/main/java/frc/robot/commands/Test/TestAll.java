// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.SmartDashboardPutData;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAll extends SequentialCommandGroup {
  /** Creates a new TestAll. */
  public TestAll() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SmartDashboardPutData("current test",  "drive"),
      new TestDrive(),
      new DriveFollowPath("systemCheckPath", 1, 1),
      new SmartDashboardPutData("current test", "intake"),
      new TestIntakeAndFeeder(),
      new SmartDashboardPutData("current test", "shooter"),
      new TestShooter(),
      new SmartDashboardPutData("current test", "hood"),
      new TestHood(),
      // new SmartDashboardPutData("current test", "climber"),
      // new TestClimber(),
      new SmartDashboardPutData("current test", "none")
    );
  }
}
