// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Sets the rpm but doesn't shoot yet */ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterSetSpeed extends InstantCommand {
  double rpm;
  private Shooter shooter;

  /** Creates a new ShooterSetSpeed. */
  public ShooterSetSpeed(int side, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = RobotContainer.shooter[side];
    addRequirements(shooter);
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPM(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
