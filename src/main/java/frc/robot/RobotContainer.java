// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CG_IntakeBalls;
import frc.robot.commands.CG_SetShooterSpeedAndAngle;
import frc.robot.commands.CG_ShootAll;
import frc.robot.commands.ClimbAllTheWay;
import frc.robot.commands.ClimberForceToTop;
import frc.robot.commands.ClimberHomePosition;
import frc.robot.commands.ClimberStop;
import frc.robot.commands.ClimberToBarRelease;
import frc.robot.commands.ClimberToBottom;
import frc.robot.commands.ClimberToTop;
import frc.robot.commands.CompressorSetEnabled;
import frc.robot.commands.DriveAutoRotate;
import frc.robot.commands.DriveSetBrakeMode;
import frc.robot.commands.DriveFeedbackWhileDisabled;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveMakeAllCurrentModuleAnglesZero;
import frc.robot.commands.DriveOneModule;
import frc.robot.commands.DriveSetModuleAngles;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.DriveXMode;
import frc.robot.commands.DriveZeroGyro;
import frc.robot.commands.FloppyArmDown;
import frc.robot.commands.FloppyArmUp;
import frc.robot.commands.HoodHome;
import frc.robot.commands.HoodOverrideRestPosition;
import frc.robot.commands.HoodSetByNetwork;
import frc.robot.commands.HoodSetPosition;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeSetColorOverride;
import frc.robot.commands.IntakeSetOverride;
import frc.robot.commands.IntakeSetPneumatic;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.LimelightManualOff;
import frc.robot.commands.LimelightManualOn;
import frc.robot.commands.LimelightPowerCycle;
import frc.robot.commands.LimelightZoneMode;
import frc.robot.commands.PurgeBall;
import frc.robot.commands.RumbleCoDriver;
import frc.robot.commands.ShooterAutoPrep;
import frc.robot.commands.ShooterSetByNetwork;
import frc.robot.commands.ShooterSetSpeed;
import frc.robot.commands.ShooterSetSpeedOverride;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.Autonomous.Auto1Ball;
import frc.robot.commands.Autonomous.Auto2Ball;
import frc.robot.commands.Autonomous.Auto2Ball2;
import frc.robot.commands.Autonomous.Auto2BallSteal;
import frc.robot.commands.Autonomous.Auto3Ball;
import frc.robot.commands.Autonomous.Auto4Ball;
import frc.robot.commands.Autonomous.Auto5Ball;
import frc.robot.commands.DriveWithGamepad.DefaultWheelStates;
import frc.robot.commands.Test.DebugAll;
import frc.robot.commands.Test.TestAll;
import frc.robot.commands.Test.TestDrive;
import frc.robot.commands.Test.TestHood;
import frc.robot.commands.Test.TestIntakeAndFeeder;
import frc.robot.commands.Test.TestShooter;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static Drive drive;
  public static Intake[] intake = { new Intake(), new Intake() };
  public static Feeder[] feeder = { new Feeder(), new Feeder() };
  public static IntakeFeeder[] intakeFeeder = { new IntakeFeeder(), new IntakeFeeder() };
  public static Shooter[] shooter = { new Shooter(), new Shooter() };
  public static Hood hood;
  public static Limelight limelight;
  public static Climber climber;
  public static Compressor compressor;
  public static ShooterSetSpeedOverride shooterOverride;
  public static SubsystemActiveTrigger isClimberRunning;

  public static final XboxController driver = new XboxController(0);
  public static final XboxController coDriver = new XboxController(1);

  public static final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
  public static final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
  public static final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
  public static final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
  public static final JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  public static final JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  public static final DPadButton driverDUp = new DPadButton(driver, DPadButton.Value.kDPadUp);
  public static final DPadButton driverDDown = new DPadButton(driver, DPadButton.Value.kDPadDown);
  public static final DPadButton driverDLeft = new DPadButton(driver, DPadButton.Value.kDPadLeft);
  public static final DPadButton driverDRight = new DPadButton(driver, DPadButton.Value.kDPadRight);
  public static final TriggerButton driverLT = new TriggerButton(driver, true);
  public static final TriggerButton driverRT = new TriggerButton(driver, false);
  public static final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
  public static final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);

  public static final JoystickButton coDriverA = new JoystickButton(coDriver, XboxController.Button.kA.value);
  public static final JoystickButton coDriverB = new JoystickButton(coDriver, XboxController.Button.kB.value);
  public static final JoystickButton coDriverX = new JoystickButton(coDriver, XboxController.Button.kX.value);
  public static final JoystickButton coDriverY = new JoystickButton(coDriver, XboxController.Button.kY.value);
  public static final JoystickButton coDriverLB = new JoystickButton(coDriver,
      XboxController.Button.kLeftBumper.value);
  public static final JoystickButton coDriverRB = new JoystickButton(coDriver,
      XboxController.Button.kRightBumper.value);
  public static final DPadButton coDriverDUp = new DPadButton(coDriver, DPadButton.Value.kDPadUp);
  public static final DPadButton coDriverDDown = new DPadButton(coDriver, DPadButton.Value.kDPadDown);
  public static final DPadButton coDriverDLeft = new DPadButton(coDriver, DPadButton.Value.kDPadLeft);
  public static final DPadButton coDriverDRight = new DPadButton(coDriver, DPadButton.Value.kDPadRight);
  public static final TriggerButton coDriverLT = new TriggerButton(coDriver, true);
  public static final TriggerButton coDriverRT = new TriggerButton(coDriver, false);
  public static final JoystickButton coDriverStart = new JoystickButton(coDriver, XboxController.Button.kStart.value);
  public static final JoystickButton coDriverBack = new JoystickButton(coDriver, XboxController.Button.kBack.value);

  public static final Timer timer = new Timer();

  SendableChooser<CommandBase> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive = new Drive();
    SmartDashboard.putData("Rotate Module", new DriveFeedbackWhileDisabled());
    SmartDashboard.putData("Zero Modules", new DriveMakeAllCurrentModuleAnglesZero());

    compressor = new Compressor();
    climber = new Climber();
    limelight = new Limelight();
    // intake[0] = new Intake(0);
    // intake[1] = new Intake(1);
    // feeder[0] = new Feeder(0);
    // feeder[1] = new Feeder(1);
    intakeFeeder[0] = new IntakeFeeder(0);
    intakeFeeder[1] = new IntakeFeeder(1);
    shooter[0] = new Shooter(0);
    shooter[1] = new Shooter(1);
    hood = new Hood();
    isClimberRunning = new SubsystemActiveTrigger(climber);
    shooterOverride = new ShooterSetSpeedOverride(true);
    SmartDashboard.putData("Front Right Control", new DriveOneModule(0));
    SmartDashboard.putData("Front Left Control", new DriveOneModule(1));
    SmartDashboard.putData("Back Left Control", new DriveOneModule(2));
    SmartDashboard.putData("Back Right Control", new DriveOneModule(3));
    SmartDashboard.putData("Turn to 0", new DriveTurnToAngle(0));
    SmartDashboard.putData("Turn to 90", new DriveTurnToAngle(90));
    SmartDashboard.putData("Hood home", new HoodHome());
    SmartDashboard.putData("Enable Compressor", new CompressorSetEnabled(true));
    SmartDashboard.putData("Disable Compressor", new CompressorSetEnabled(false));
    SmartDashboard.putData("Floppy Arm Down", new FloppyArmDown());
    SmartDashboard.putData("Floppy Arm Up", new FloppyArmUp());
    SmartDashboard.putData("Climber Home", new ClimberHomePosition());
    SmartDashboard.putData("Zero 202", new DriveZeroGyro(158));
    SmartDashboard.putData("Hood 0", new HoodSetPosition(0));
    SmartDashboard.putData("Hood 5", new HoodSetPosition(5));
    SmartDashboard.putData("Hood 15", new HoodSetPosition(15));
    SmartDashboard.putData("Hood 20", new HoodSetPosition(20));
    SmartDashboard.putData("Hood Home Fast", new HoodHome(0.8));
    SmartDashboard.putData("Hood Home Lightning", new HoodHome(1));
    SmartDashboard.putData("Limelight manual on", new LimelightManualOn());
    SmartDashboard.putData("Set Modules 0", new DriveSetModuleAngles(0));
    SmartDashboard.putData("Set Modules 90", new DriveSetModuleAngles(90));
    SmartDashboard.putData("Override shooters", new ShooterSetSpeedOverride(true));
    SmartDashboard.putData("Override Shooters off", new ShooterSetSpeedOverride(false));
    SmartDashboard.putData("Test Everything", new TestAll());
    SmartDashboard.putData("Test Drive", new TestDrive());
    SmartDashboard.putData("Test Intake and Feeder", new TestIntakeAndFeeder());
    SmartDashboard.putData("Test Shooter", new TestShooter());
    SmartDashboard.putData("Test Hood", new TestHood());
    SmartDashboard.putData("Limelight power cycle", new LimelightPowerCycle());
    SmartDashboard.putData("Climber Up", new ClimberToTop());
    SmartDashboard.putData("Climber Down", new ClimberToBottom());
    SmartDashboard.putData("Toggle Color Override On", parallel(new IntakeSetColorOverride(0, true), new IntakeSetColorOverride(1, true)));
    SmartDashboard.putData("Toggle Color Override Off", parallel(new IntakeSetColorOverride(0, false), new IntakeSetColorOverride(1, false)));
    SmartDashboard.putData("Enable Debug Mode", new DebugAll());
    SmartDashboard.putData("Climber Coast", new ClimberStop(false));
    SmartDashboard.putData("Climber Brake", new ClimberStop(true));
    SmartDashboard.putData("Drive X", new DriveFollowPath("xPath", 3, 4));
    SmartDashboard.putData("Drive Y", new DriveFollowPath("yPath", 3, 4));
    SmartDashboard.putData("Heading", new DriveFollowPath("forwardMoveRight", 3, 4));
    SmartDashboard.putData("Rotation", new DriveFollowPath("forwardRotateLeft", 3, 4));
    SmartDashboard.putData("Set Drive Brake Mode", new DriveSetBrakeMode(true));
    SmartDashboard.putData("Set Drive Coast Mode", new DriveSetBrakeMode(false));
    SmartDashboard.putData("Set Zone Mode On", new LimelightZoneMode(true));
    SmartDashboard.putData("Set Zone Mode Off", new LimelightZoneMode(false));
    SmartDashboard.putData("Limelight Manual Off", new LimelightManualOff());
    // drive.setDefaultCommand(new DriveWithGamepad(true, true,
    //   new DefaultWheelStates(
    //     new double[]{135, 225, 315, 45})));
    drive.setDefaultCommand(new DriveWithGamepad(true, false));
    // hood.setDefaultCommand(new HoodHome());
    // hood.setDefaultCommand(new HoodDPad());
    // SmartDashboard.putData(climber);

    autoChooser.setDefaultOption("No Auto Selected", new WaitCommand(1.0));
    autoChooser.addOption("5 Ball Auto", new Auto5Ball());
    autoChooser.addOption("4 Ball Auto", new Auto4Ball());
    autoChooser.addOption("3 Ball Auto", new Auto3Ball());
    autoChooser.addOption("2 Ball Auto (Feed Me)", new Auto2Ball());
    autoChooser.addOption("2 Ball Auto (Feed Me, Steal)", new Auto2BallSteal());
    autoChooser.addOption("2 Ball Auto (Steal)", new Auto2Ball2());
    autoChooser.addOption("Auto for bottoms and third picks (Lizzie's Auto)", new Auto1Ball());

    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**`
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    double startSpeed = 3800;
    double startAngle = 24.5;
    double lowFenderSpeed = Constants.SHOOTER_REST_SPEED;
    double highFenderSpeed = 3200; // 3300
    double lowFenderAngle = Constants.HOOD_REST_POSITION;
    double highFenderAngle = 7.05;
    double setSpeed = 3685;
    double setAngle = 25.2;
    /*================  Driver Controls  ================*/
    driverB.and(isClimberRunning.negate())
      .whileActiveContinuous(parallel(new PurgeBall(0), new PurgeBall(1)));
    // driverY.and(isClimberRunning.negate())
    //   .whileActiveContinuous(parallel(new ShooterSetSpeed(0, highFenderSpeed), new ShooterSetSpeed(1, highFenderSpeed), new HoodSetPosition(highFenderAngle)))
    //   .whenInactive(parallel(new ShooterSetSpeed(0, lowFenderSpeed), new ShooterSetSpeed(1, lowFenderSpeed)));
    driverY.and(isClimberRunning.negate())
        .whenActive(new HoodHome());
    driverLB.whenActive(new DriveZeroGyro());
    driverRB.and(isClimberRunning.negate())
      .whenActive(new IntakeBall(0))
      .whenActive(new IntakeBall(1))
      .whenInactive(new IntakeStop(0))
      .whenInactive(new IntakeStop(1));
    driverA.and(isClimberRunning.negate())
      .whenActive(new IntakeBall(0))
      .whenActive(new IntakeBall(1))
      .whenInactive(new IntakeStop(0))
      .whenInactive(new IntakeStop(1));
    driverX.and(isClimberRunning.negate())
      .whileActiveContinuous(new DriveXMode());
    // driverLT.and(isClimberRunning.negate())
    //   .whileActiveContinuous(parallel(new DriveAutoRotate(), new ShooterSetSpeed(0, setSpeed), new ShooterSetSpeed(1, setSpeed), new HoodSetPosition(setAngle), new HoodOverrideRestPosition(true)))
    //   .whenInactive(parallel(new ShooterStop(0), new ShooterStop(1), new HoodOverrideRestPosition(false)));

    driverLT.and(isClimberRunning.negate())
      .whileActiveContinuous(parallel(new DriveAutoRotate(), new ShooterAutoPrep()))
      .whenInactive(parallel(new ShooterStop(0), new ShooterStop(1)));
    driverDUp.and(isClimberRunning.negate())
    .whenActive(parallel(
        new IntakeSetOverride(0, true),
        new IntakeSetOverride(1, true),
        new IntakeSetPneumatic(0, false),
        new IntakeSetPneumatic(1, false),
        new IntakeBall(0),
        new IntakeBall(1)
      ))
      .whenInactive(parallel(
        new IntakeStop(0),
        new IntakeStop(1),
        new IntakeSetOverride(0, false),
        new IntakeSetOverride(1, false)
      ));
    driverDLeft.and(isClimberRunning.negate())
      .whileActiveContinuous(new IntakeBall(0));
    driverDRight.and(isClimberRunning.negate())
      .whileActiveContinuous(new IntakeBall(1));
    driverRT.and(isClimberRunning.negate()).and(coDriverDUp.negate())
      .whileActiveContinuous(new CG_ShootAll()); 
    driverRT.and(isClimberRunning.negate()).and(coDriverDUp)
      .whileActiveContinuous(new CG_ShootAll(0.6, 0.6));

    /* For shooter tuning, do not use in comp */
    // driverLT
    //   .whileActiveContinuous(parallel(new ShooterSetByNetwork(0), new ShooterSetByNetwork(1), new HoodSetByNetwork()))
    //   .whenInactive(parallel(new ShooterStop(0), new ShooterStop(1)));
      
    /*================ CoDriver Controls ================*/
    coDriverA.and(isClimberRunning.negate())
      .whenActive(new IntakeBall(0))
      .whenActive(new IntakeBall(1))
      .whenInactive(new IntakeStop(0))
      .whenInactive(new IntakeStop(1));
    coDriverB.whileActiveContinuous(parallel(new PurgeBall(0), new PurgeBall(1)));
    coDriverX.and(isClimberRunning.negate())
      .whenActive(parallel(
        new IntakeSetOverride(0, true),
        new IntakeSetOverride(1, true),
        new IntakeSetPneumatic(0, false),
        new IntakeSetPneumatic(1, false),
        new IntakeBall(0),
        new IntakeBall(1)
      ))
      .whenInactive(parallel(
        new IntakeSetOverride(0, false),
        new IntakeSetOverride(1, false)
      ));
    coDriverY.and(isClimberRunning.negate())
      .whileActiveContinuous(new CG_SetShooterSpeedAndAngle(highFenderSpeed, lowFenderSpeed, highFenderAngle))
      .whenInactive(parallel(new ShooterStop(0), new ShooterStop(1)));
    coDriverStart.and(coDriverBack)
      .toggleWhenActive(parallel(new ClimbAllTheWay(), new DriveWithGamepad(true, false), new ShooterSetSpeedOverride(true)))
      .whenActive(new RumbleCoDriver(0.5));
    coDriverLB 
      .whenPressed(parallel(new IntakeSetColorOverride(0, true), new IntakeSetColorOverride(1, true)));
    coDriverRB
      .whenPressed(parallel(new IntakeSetColorOverride(0, false), new IntakeSetColorOverride(1, false)));
    // coDriverLT.and(isClimberRunning.negate())
    //   .whileActiveContinuous(parallel(new DriveAutoRotate(), new ShooterSetSpeed(0, setSpeed), new ShooterSetSpeed(1, setSpeed), new HoodSetPosition(setAngle), new HoodOverrideRestPosition(true)))
    //   .whenInactive(parallel(new ShooterStop(0), new ShooterStop(1), new HoodOverrideRestPosition(false)));
    coDriverLT.and(isClimberRunning.negate())
      .whileActiveContinuous(parallel(new DriveAutoRotate(), new ShooterAutoPrep()));
    coDriverRT.and(isClimberRunning.negate()).and(coDriverDUp.negate())
      .whileActiveContinuous(new CG_ShootAll());
    coDriverRT.and(isClimberRunning.negate()).and(coDriverDUp)
      .whileActiveContinuous(new CG_ShootAll(0.6, 0.6)); 
    coDriverDUp.and(isClimberRunning.negate())
      .whileActiveContinuous(new CG_SetShooterSpeedAndAngle(highFenderSpeed, highFenderAngle))
      .whenInactive(parallel(new ShooterStop(0), new ShooterStop(1)));
    coDriverDDown.and(isClimberRunning.negate())
      .whileActiveContinuous(new CG_SetShooterSpeedAndAngle(lowFenderSpeed, lowFenderAngle))
      .whenInactive(parallel(new ShooterStop(0), new ShooterStop(1)));
    coDriverDLeft.and(isClimberRunning.negate())
      .whileActiveContinuous(new IntakeBall(0));
    coDriverDRight.and(isClimberRunning.negate())
      .whileActiveContinuous(new IntakeBall(1));

    /*================  Manual Climbing  ================*/
    coDriverDUp.or(driverDUp).and(isClimberRunning)
      .whenActive(new ClimberForceToTop());
    coDriverDDown.or(driverDDown).and(isClimberRunning)
      .whenActive(new ClimberToBottom());
    coDriverY.or(driverY).and(isClimberRunning)
      .whenActive(new FloppyArmUp());
    coDriverX.or(driverX).and(isClimberRunning)
      .whenActive(new FloppyArmDown());
    coDriverB.or(driverB).and(isClimberRunning)
      .whenActive(new ClimberStop(false));
    
    SmartDashboard.putNumber("shooter speed", startSpeed);
    SmartDashboard.putNumber("Hood Set Position", startAngle);
  }

  public static void setCoDriverRumble(double outputLeft, double outputRight) {
    coDriver.setRumble(RumbleType.kLeftRumble, outputLeft);
    coDriver.setRumble(RumbleType.kRightRumble, outputRight);
  }

  /**
   * Factory method for {@link ParallelCommandGroup}, included for brevity/convenience.
   *
   * @param commands the commands to include
   * @return the command group
   */
  public static CommandGroupBase parallel(Command... commands) {
    return new ParallelCommandGroup(commands);
  }

  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

  public static double getDriverLeftTrigger() {
    return driver.getLeftTriggerAxis();
  }

  public static double getDriverRightTrigger() {
    return driver.getRightTriggerAxis();
  }

  public static double getDriverLeftStickX() {
    return Math.abs(driver.getLeftX()) < Constants.DRIVE_DEADZONE ? 0 : driver.getLeftX();
  }

  public static double getDriverLeftStickY() {
    return Math.abs(driver.getLeftY()) < Constants.DRIVE_DEADZONE ? 0 : driver.getLeftY();
  }

  public static double getDriverRightStickX() {
    return Math.abs(driver.getRightX()) < Constants.DRIVE_DEADZONE ? 0 : driver.getRightX();
  }

  public static double getDriverRightStickY() {
    return Math.abs(driver.getRightY()) < Constants.DRIVE_DEADZONE ? 0 : driver.getRightY();
  }

  public static int getDriverDPad() {
    return driver.getPOV();
  }

  public static class TriggerButton extends Trigger {
    boolean isLeftTrigger;
    XboxController controller;

    public TriggerButton(XboxController controller, boolean isLeftTrigger) {
      this.isLeftTrigger = isLeftTrigger;
      this.controller = controller;
    }

    @Override
    public boolean get() {
      if (isLeftTrigger)
        return controller.getLeftTriggerAxis() >= 0.5;
      else {
        return controller.getRightTriggerAxis() >= 0.5;
      }
    }
  }

  static class DPadButton extends Button {
    private final int dPadDegree;
    private final XboxController controller;

    public enum Value {
      kDPadRight, kDPadUpRight, kDPadUp, kDPadUpLeft, kDPadLeft, kDPadDownLeft, kDPadDown, kDPadDownRight,
    }

    /**
     * Creates a dpad button
     *
     * @param controller the controller to attach the button to
     * @param value      the dpad value
     */

    public DPadButton(XboxController controller, Value value) {
      this.controller = controller;
      switch (value) {
        case kDPadRight:
          this.dPadDegree = 90;
          break;
        case kDPadUpRight:
          this.dPadDegree = 45;
          break;
        case kDPadUp:
          this.dPadDegree = 0;
          break;
        case kDPadUpLeft:
          this.dPadDegree = 315;
          break;
        case kDPadLeft:
          this.dPadDegree = 270;
          break;
        case kDPadDownLeft:
          this.dPadDegree = 225;
          break;
        case kDPadDown:
          this.dPadDegree = 180;
          break;
        case kDPadDownRight:
          this.dPadDegree = 135;
          break;
        default:
          throw new AssertionError("Illegal value" + value);
      }
    }

    @Override
    public boolean get() {
      return controller.getPOV() == dPadDegree;
    }
  }

}