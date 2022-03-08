// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Zeros Swerve module angles */

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveMakeAllCurrentModuleAnglesZero extends CommandBase {
  Timer timer = new Timer();

  /** Creates a new DriveMakeAllCurrentModuleAnglesZero. */
  public DriveMakeAllCurrentModuleAnglesZero() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if (!interrupted && DriverStation.isDisabled()) {
      RobotContainer.drive.setRotationOffsets(90);
    }else if(interrupted){
      System.out.println("WARNING: RESET COMMAND INTERRUPTED - RESET FAILED");
    }else if(!DriverStation.isDisabled()){
      System.out.println("WARNING: ROBOT NOT DISABLED (DISABLE ROBOT!) - RESET FAILED");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= 10) || !DriverStation.isDisabled();
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
