package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonSRX rotationMotor;
    private double rotationOffset;

    SwerveModule(int driveChannel, int rotationChannel, double rotationOffset) {
        this.rotationOffset = rotationOffset;
        driveMotor = new TalonFX(driveChannel);
        driveMotor.configFactoryDefault();
        rotationMotor = new TalonSRX(rotationChannel);
        rotationMotor.configFactoryDefault();
        rotationMotor.setNeutralMode(NeutralMode.Coast);

        driveMotor.setInverted(false);
        driveMotor.setSensorPhase(false);
        driveMotor.config_kP(0, Constants.SWERVE_DRIVE_P);
        driveMotor.config_kI(0, Constants.SWERVE_DRIVE_I);
        driveMotor.config_kD(0, Constants.SWERVE_DRIVE_D);
        driveMotor.config_kF(0, 1023.0 / (Constants.SWERVE_MAX_VELOCITY_METERS / Constants.SWERVE_METERS_PER_PULSE));
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
        // driveMotor.configClosedloopRamp(0.5);
        // driveMotor.setNeutralMode(NeutralMode.Coast);

        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        // rotationMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_DEGREES_PER_PULSE,
        // 0, 10);
        // rotationMotor.configSelectedFeedbackCoefficient(Constants.SWERVE_DEGREES_PER_PULSE,
        // 1, 10);
        rotationMotor.configVoltageCompSaturation(12.5);
        rotationMotor.enableVoltageCompensation(true);
        rotationMotor.configFeedbackNotContinuous(true, 10);
        rotationMotor.setInverted(true);
        rotationMotor.setSensorPhase(true);
        rotationMotor.config_kP(0, Constants.SWERVE_ROTATION_P);
        rotationMotor.config_kI(0, Constants.SWERVE_ROTATION_I);
        rotationMotor.config_IntegralZone(0, Constants.SWERVE_ROTATION_I_ZONE);
        rotationMotor.config_kD(0, Constants.SWERVE_ROTATION_D);
        // rotationMotor.config_kF(0, Constants.SWERVE_ROTATION_KV);
        // rotationMotor.configMotionCruiseVelocity(Constants.SWERVE_ROTATION_MAX_VELOCITY);
        // rotationMotor.configMotionAcceleration(Constants.SWERVE_ROTATION_MAX_ACCEL);
        rotationMotor.configPeakCurrentLimit(0);
        rotationMotor.configPeakCurrentDuration(0);
        rotationMotor.configAllowableClosedloopError(0, 1 / Constants.SWERVE_DEGREES_PER_PULSE);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10);
    }

    /**
     * Sets the swerve module's drive motor to brake mode or coast mode
     * 
     * @param enable
     *               <ul>
     *               <li><code>true</code> - brake mode
     *               <li><code>false</code> - coast mode
     */
    public void enableBrakeMode(boolean enable) {
        if (enable) {
            driveMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            driveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    /**
     * Resets the module's drive and rotation encoders
     */
    public void resetEncoders() {
        rotationMotor.setSelectedSensorPosition(0);
        driveMotor.setSelectedSensorPosition(0);
    }

    /**
     * Gets the module's absolute rotation relative to a defined zero in degrees
     * 
     * @return The module's angle in degrees
     */
    public double getAbsoluteRotation() {
        return (rotationMotor.getSelectedSensorPosition(1) * Constants.SWERVE_DEGREES_PER_PULSE) - 180 - rotationOffset;
    }

    /**
     * Gets the module's rotation relative to the most recent encoder reset
     * 
     * @return The module's relative angle in degrees
     */
    public double getRelativeRotation() {
        return rotationMotor.getSelectedSensorPosition(0) * Constants.SWERVE_DEGREES_PER_PULSE;
    }

    /**
     * Sets the drive motor based on specified control mode and speed
     * 
     * @param controlMode Specifies how 'outputValue' should be interpreted by the
     *                    motor
     * @param outputValue The value the drive motor will be set to. Based on the
     *                    controlMode
     */
    public void setDriveMotor(ControlMode controlMode, double outputValue) {
        driveMotor.set(controlMode, outputValue);
    }

    /**
     * Gets the swerve module's current velocity in meters and angle in degrees
     * encapsulated
     * in a SwerveModuleState
     * 
     * @return The swerve module's current state
     */
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAbsoluteRotation()));
    }

    /**
     * Get the swerve module's current velocity in meters
     * 
     * @return The drive motor's current velocity in meters
     */
    public double getVelocity() {
        return driveMotor.getSelectedSensorVelocity() * 10 * Constants.SWERVE_METERS_PER_PULSE;
    }

    /**
     * Sets the rotation of the swerve module to a specified angle in degrees
     * 
     * @param degrees the angle the module will be set to
     */
    public void setRotationPosition(double degrees) {
        double currentPosAbs = getAbsoluteRotation();
        double currentPosRel = getRelativeRotation();
        double delta = degrees - currentPosAbs;
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }
        rotationMotor.set(ControlMode.Position, (currentPosRel + delta) / Constants.SWERVE_DEGREES_PER_PULSE);
    }

    /**
     * Sets the rotation and velocity of the swerve module based on a swerve module
     * state
     * 
     * @param state The swerve module state to target, with speed in meters per
     *              second and angle in degrees
     */
    public void setSwerveModuleState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
                Rotation2d.fromDegrees(getAbsoluteRotation()));
        if (Math.abs(optimizedState.speedMetersPerSecond) > 0) {
            setRotationPosition(optimizedState.angle.getDegrees());
        }
        setDriveMotor(ControlMode.Velocity, optimizedState.speedMetersPerSecond / Constants.SWERVE_METERS_PER_PULSE);
    }

    /**
     * Sets the Swerve Module's zero position to its current angle and return the
     * new offset
     * 
     * @return the new offset in degrees
     */
    public double updateRotationOffset() {
        rotationOffset += getAbsoluteRotation();
        return rotationOffset;
    }

    /**
     * Gets the Swerve Modules current rotation offset
     * 
     * @return the current offset in degrees
     */
    public double getRotationOffset() {
        return rotationOffset;
    }

    /**
     * Sets both drive and rotation motor to zero velocity
     */
    public void setStopMotor() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        rotationMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Sets the rotation speed of the module by percent
     * 
     * @param speed the percent to run the rotation motor at
     */
    public void setRotationSpeed(double speed) {
        rotationMotor.set(ControlMode.PercentOutput, speed);
    }

}