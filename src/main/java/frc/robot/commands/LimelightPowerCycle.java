// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LimelightPowerCycle extends CommandBase {
  Timer time;
  /**
   * A command to turn off then on again the LImelight 
   * by switching the PDH's switchablle power output.
   */
  public LimelightPowerCycle() {
    addRequirements(RobotContainer.limelight);
    time = new Timer();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.setLimelightPower(false);
    time.reset();
    time.start();
  }

  public void end(boolean interrupted){
    RobotContainer.limelight.setLimelightPower(true);
  }

  public boolean isFinished(){
    return time.hasElapsed(1.0);
  }

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }
}