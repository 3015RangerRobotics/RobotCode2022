// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RumbleCoDriver extends CommandBase {
  Timer timer = new Timer();
  double timeout;
  /** 
   * Lightly rumbles the coDriver gamepad for the timeOut amount
   * @param timeout time in seconds
   */
  public RumbleCoDriver(double timeout) {
    this.timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    RobotContainer.setCoDriverRumble(0.25, 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.setCoDriverRumble(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeout);
  }
}
