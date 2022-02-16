// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterSetByNetwork extends InstantCommand {
  double rpm;
  private Shooter shooter;

  /** Creates a new ShooterSetSpeed. */
  public ShooterSetByNetwork(int side) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = RobotContainer.shooter[side];
    addRequirements(shooter);
    SmartDashboard.putNumber("shooter speed", 0);
    this.rpm = SmartDashboard.getNumber("shooter speed", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.rpm = SmartDashboard.getNumber("shooter speed", 0);
    SmartDashboard.putNumber("shooter speed", rpm); // Update network table for when it gets annoying
    shooter.setRPM(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
