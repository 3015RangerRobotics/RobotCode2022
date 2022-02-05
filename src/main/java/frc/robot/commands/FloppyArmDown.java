// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.SolenoidPosition;

public class FloppyArmDown extends CommandBase {
  private double endAngle;
  private boolean hasEndCondition;

  public FloppyArmDown() {
    this(0, false);
  }

  public FloppyArmDown(double endAngle) {
    this(endAngle, true);
  }

  /** Creates a new FloppyArmDown. */
  private FloppyArmDown(double endAngle, boolean hasEndCondition) {
    this.endAngle = endAngle;
    this.hasEndCondition = hasEndCondition;
    addRequirements(RobotContainer.climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.setSolenoidPosition(SolenoidPosition.kDown);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atAngle = (Math.abs((RobotContainer.climber.getArmAngle() + RobotContainer.drive.getYAngle())
        - endAngle) < Constants.CLIMBER_FLOPPY_POSITION_TOLERANCE);
    boolean isStopped = (Math.abs(RobotContainer.climber.getArmSpeed()) < Constants.CLIMBER_FLOPPY_SPEED_TOLERANCE);
    return !hasEndCondition || (atAngle && isStopped);

  }
}
