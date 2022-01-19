package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//Added WPI_ prefix for compatabilitiy with WPI lib classes
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
    private WPI_TalonFX driveMotor;
    private WPI_TalonSRX rotationMotor;
    private double rotationOffset;

    SwerveModule(int driveChannel, int rotationChannel, double rotationOffset) {
        this.rotationOffset = rotationOffset;
        driveMotor = new WPI_TalonFX(driveChannel);
        driveMotor.configFactoryDefault();
        rotationMotor = new WPI_TalonSRX(rotationChannel);
        rotationMotor.configFactoryDefault();
        rotationMotor.setNeutralMode(NeutralMode.Coast);

        driveMotor.setInverted(false);
        driveMotor.setSensorPhase(false);
        driveMotor.config_kP(0, Constants.SWERVE_DRIVE_P);
        driveMotor.config_kI(0, Constants.SWERVE_DRIVE_I);
        driveMotor.config_kD(0, Constants.SWERVE_DRIVE_D);
        driveMotor.config_kF(0, 1023.0 / (Constants.SWERVE_MAX_VELOCITY / Constants.SWERVE_METERS_PER_PULSE));
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
//        driveMotor.configClosedloopRamp(0.5);
//        driveMotor.setNeutralMode(NeutralMode.Coast);

        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
//        rotationMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_DEGREES_PER_PULSE, 0, 10);
//        rotationMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_DEGREES_PER_PULSE, 1, 10);
        rotationMotor.configVoltageCompSaturation(12.5);
        rotationMotor.enableVoltageCompensation(true);
        rotationMotor.configFeedbackNotContinuous(true, 10);
        rotationMotor.setInverted(true);
        rotationMotor.setSensorPhase(true);
        rotationMotor.config_kP(0, Constants.SWERVE_ROTATION_P);
        rotationMotor.config_kI(0, Constants.SWERVE_ROTATION_I);
        rotationMotor.config_IntegralZone(0, Constants.SWERVE_ROTATION_I_ZONE);
        rotationMotor.config_kD(0, Constants.SWERVE_ROTATION_D);
//        rotationMotor.config_kF(0, Constants.SWERVE_ROTATION_KV);
//        rotationMotor.configMotionCruiseVelocity(Constants.SWERVE_ROTATION_MAX_VELOCITY);
//        rotationMotor.configMotionAcceleration(Constants.SWERVE_ROTATION_MAX_ACCEL);
//        rotationMotor.configAllowableClosedloopError(0, 1/Constants.SWERVE_DEGREES_PER_PULSE);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10);
    }

    public void enableBrakeMode(boolean enable){
        if (enable){
            driveMotor.setNeutralMode(NeutralMode.Brake);
        }
        else {
            driveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }
    public void resetEncoders() {
        rotationMotor.setSelectedSensorPosition(0);
        driveMotor.setSelectedSensorPosition(0);
    }

    public double getAbsoluteRotation(){
        return (rotationMotor.getSelectedSensorPosition(1) * Constants.SWERVE_DEGREES_PER_PULSE) - 180 - rotationOffset;
    }

    public double getRelativeRotation(){
        return rotationMotor.getSelectedSensorPosition(0) * Constants.SWERVE_DEGREES_PER_PULSE;
    }

    public void setDriveMotor(ControlMode controlMode, double outputValue) {
        driveMotor.set(controlMode, outputValue);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAbsoluteRotation()));
    }

    public double getVelocity(){
        return driveMotor.getSelectedSensorVelocity() * 10 * Constants.SWERVE_METERS_PER_PULSE;
    }

    public void setRotationPosition(double degrees){
        double currentPosAbs = getAbsoluteRotation();
        double currentPosRel = getRelativeRotation();
        double delta = degrees - currentPosAbs;
        if(delta > 180){
            delta -= 360;
        }else if(delta < -180){
            delta += 360;
        }
        rotationMotor.set(ControlMode.Position, (currentPosRel + delta) / Constants.SWERVE_DEGREES_PER_PULSE);
    }

    public void setSwerveModuleState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAbsoluteRotation()));
        if(Math.abs(optimizedState.speedMetersPerSecond) > 0){
            setRotationPosition(optimizedState.angle.getDegrees());
        }
        setDriveMotor(ControlMode.Velocity, optimizedState.speedMetersPerSecond / Constants.SWERVE_METERS_PER_PULSE);
//        System.out.println(optimizedState.speedMetersPerSecond);
    }

}