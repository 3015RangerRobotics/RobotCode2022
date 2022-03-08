// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_IntakeBalls extends ParallelCommandGroup {

  public CG_IntakeBalls() {
    this(true);
  }

  /** Creates a new CG_IntakeBalls. */
  public CG_IntakeBalls(boolean affectPneumatic) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeBall(0, affectPneumatic),
      new IntakeBall(1, affectPneumatic)
    );
  }
}
