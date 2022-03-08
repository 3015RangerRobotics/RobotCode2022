// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_ShootAll extends ParallelCommandGroup {

  public CG_ShootAll() {
    this(0.25, 0.25);
  }

  /** Creates a new CG_ShootAll. */
  public CG_ShootAll(double leftRightDelay, double sameSideDelay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootBalls(0, 0, sameSideDelay, 0),
      new ShootBalls(1, 0, sameSideDelay, leftRightDelay)
    );
  }
}
