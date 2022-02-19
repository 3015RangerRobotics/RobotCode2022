// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.LEDMode;

public class DriveAutoRotate extends CommandBase {
  double setPoint;
  PIDController rotationController;
  Timer timer = new Timer();
  boolean hasHadTarget;
  /** Creates a new driveAutoRotate. */
  public DriveAutoRotate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    rotationController = new PIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D);
    rotationController.setTolerance(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.setLEDMode(LEDMode.LED_ON);
    timer.start();
    timer.reset();
    hasHadTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double leftStickY = RobotContainer.getDriverLeftStickY();
      double leftStickX = RobotContainer.getDriverLeftStickX();
      double rightStickX = RobotContainer.getDriverRightStickX() * 0.75;

      SmartDashboard.putNumber("Targeting Error", rotationController.getPositionError());
      if (RobotContainer.limelight.hasTarget()) {
        setPoint = RobotContainer.drive.getTotalAngleInDegrees()-RobotContainer.limelight.getCorrectedAngleX();
      }
      else if(!hasHadTarget && timer.hasElapsed(.2) && !timer.hasElapsed(1)) {
        Translation2d relativePos = Constants.DRIVE_GOAL_POSITION.minus(RobotContainer.drive.getPoseMeters().getTranslation());
        setPoint = -1*Math.toDegrees(Math.atan2(relativePos.getY(), relativePos.getX()));
      }
      
    //        if(Math.abs(rotationController.getPositionError()) <= Constants.DRIVE_TARGETING_I_ZONE){
    //            rotationController.setI(Constants.DRIVE_TARGETING_CONTROLLER_I);
    //        }else{
    //            rotationController.setI(0);
    //        }
      double output;
      if(Math.abs(rightStickX) > .1){
        output = rightStickX * Constants.DRIVE_MAX_ANGULAR_VELOCITY;
      }else{
        output = rotationController.calculate(RobotContainer.drive.getTotalAngleInDegrees(), setPoint);
        SmartDashboard.putNumber("Targeting Output", output);
        if(!rotationController.atSetpoint() && leftStickX == 0 && leftStickY == 0) {
            if(output < 0) output = Math.min(-Constants.DRIVE_ROTATION_MIN_VELOCITY, output);
            else output = Math.max(Constants.DRIVE_ROTATION_MIN_VELOCITY, output);
        }
      }
        
//        System.out.println(output);
      RobotContainer.drive.drive(-leftStickY * Constants.SWERVE_MAX_VELOCITY_METERS, leftStickX * Constants.SWERVE_MAX_VELOCITY_METERS, output, true);
      hasHadTarget = RobotContainer.limelight.hasTarget();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
