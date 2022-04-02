// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Test.TestAll;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  NetworkTableEntry cameraSelection;
  int hRes = 320;
  int vRes = 240;
  int fps = 15;

  Timer matchTimer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();
    camera1 = CameraServer.startAutomaticCapture("Left Camera", 0);
    // camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // camera1.setPixelFormat(PixelFormat.kMJPEG);
    camera1.setVideoMode(PixelFormat.kMJPEG, hRes, vRes, fps);

    camera2 = CameraServer.startAutomaticCapture("Right Camera", 1);
    // camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // camera2.setPixelFormat(PixelFormat.kMJPEG);
    camera2.setVideoMode(PixelFormat.kMJPEG, hRes, vRes, fps);

    server = CameraServer.getServer();

    server.setSource(camera1);

    SmartDashboard.putString("active camera", "L");

    matchTimer = new Timer();
    matchTimer.reset();
    matchTimer.stop();

    // cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // if (timer.hasElapsed(10) && !camera2Started) {
    //   camera2Started = true;
    //   camera2 = CameraServer.startAutomaticCapture(1);
    //   camera2.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
    // }
    if (RobotContainer.coDriverDLeft.get() || RobotContainer.driverDLeft.get()) {
      camera2.setConnectionStrategy(ConnectionStrategy.kForceClose);
      camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      server.setSource(camera1);
      SmartDashboard.putString("active camera", "L");
      // cameraSelection.setString(camera1.getName());
    }
    if (RobotContainer.driverDRight.get() || RobotContainer.coDriverDRight.get()) {
      camera1.setConnectionStrategy(ConnectionStrategy.kForceClose);
      camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      server.setSource(camera2);
      SmartDashboard.putString("active camera", "R");
      // cameraSelection.setString(camera2.getName());
    }
    SmartDashboard.putString("alliance", DriverStation.getAlliance() == Alliance.Red ? "red" : "blue");
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    SmartDashboard.putBoolean("isEnabled", false);
    SmartDashboard.putBoolean("confirm climb", false);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    SmartDashboard.putBoolean("isEnabled", true);
    // SmartDashboard.getEntry("Override shooters/running").setBoolean(false);
    RobotContainer.shooter[0].setMinSpeedOverride(false);
    RobotContainer.shooter[1].setMinSpeedOverride(false);
    RobotContainer.drive.setBrakeModes(new boolean[] {false, true, false, true});
    matchTimer.reset();
    matchTimer.start();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("time", (int) Math.round(15 - matchTimer.get()));
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putBoolean("isEnabled", true);
    //matchTimer.reset();
    //matchTimer.start();
    RobotContainer.drive.setBrakeModes(new boolean[] {false, true, false, true});
    RobotContainer.intakeFeeder[0].setPneumaticOverride(false);
    RobotContainer.intakeFeeder[1].setPneumaticOverride(false);
    RobotContainer.hood.overriderRestPosition(false);
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("time", (int) Math.round(150 - matchTimer.get()));

    // if (RobotContainer.shooter[0].isInUse() && RobotContainer.shooter[0].isPrimed() && RobotContainer.shooter[1].isPrimed()) {
    //   RobotContainer.setCoDriverRumble(0.2, 0.2);
    // } else {
    //   RobotContainer.setCoDriverRumble(0, 0);
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    new TestAll().schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
