package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    NetworkTable powerTable = NetworkTableInstance.getDefault().getTable("power");
    private TalonFX climberMotor;
    private DoubleSolenoid secondaryArm;
    private DigitalInput bottomLimit;
    private Encoder armEncoder;
    private boolean hasBeenHomed;

    private boolean debug;

    /**
     * <ul>
     * <li>kNeutral - disables solenoid pressure
     * <li>kUp - sets the solenoid to the up position
     * <li>kDown - sets the solenoid to the down position
     */
    public enum SolenoidPosition {
        kNeutral,
        kUp,
        kDown
    }

    public Climber() {
        climberMotor = new TalonFX(Constants.CLIMBER_MOTOR);
        climberMotor.configVoltageCompSaturation(12.5);
        climberMotor.enableVoltageCompensation(true);
        climberMotor.setInverted(true);
        climberMotor.setSensorPhase(false);
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.config_kP(0, Constants.CLIMBER_RAISE_P);
        climberMotor.config_kI(0, Constants.CLIMBER_RAISE_I);
        climberMotor.config_kD(0, Constants.CLIMBER_RAISE_D);
        climberMotor.config_kF(0, Constants.CLIMBER_RAISE_F);
        climberMotor.configAllowableClosedloopError(0, 0, 20);
        climberMotor.config_kP(1, Constants.CLIMBER_LOWER_P);
        climberMotor.config_kI(1, Constants.CLIMBER_LOWER_I);
        climberMotor.config_kD(1, Constants.CLIMBER_LOWER_D);
        climberMotor.config_kF(1, Constants.CLIMBER_LOWER_F);
        // climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen, deviceID)

        /* Might need to change pneumatics module type */
        secondaryArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_REVERSE,
                Constants.CLIMBER_SOLENOID_FORWARD);

        bottomLimit = new DigitalInput(Constants.CLIMBER_BOTTOM_SWITCH);

        armEncoder = new Encoder(Constants.ENCODER_INPUT_A, Constants.ENCODER_INPUT_B);
        armEncoder.setDistancePerPulse(Constants.CLIMBER_ARM_DEGREES_PER_PULSE);
        hasBeenHomed = false;
    }

    public void periodic() {
        if (debug) {
            SmartDashboard.putBoolean("Bottom climber switch", getBottomLimit());
            SmartDashboard.putNumber("Climber position", getClimberPos());
            powerTable.getEntry("Climber Current").setDouble(climberMotor.getSupplyCurrent());
        }
    }

    /**
     * Moves the climber to its current zero position
     */
    public void homeClimber() {
        climberMotor.set(ControlMode.Position, 0);
    }

    public boolean hasBeenHomed() {
        return hasBeenHomed;
    }

    public void setHasBeenHomed() {
        hasBeenHomed = true;
    }

    public void setBrakeMode(boolean enable) {
        climberMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Sets the climber motor's position in meters
     * 
     * @param position the position to move to
     */
    public void setClimberPos(double position) {
        // position = Math.max(Math.min(position, 0),
        // Constants.CLIMBER_MAX_HEIGHT_METERS / Constants.CLIMBER_METERS_PER_PULSE);
        climberMotor.set(ControlMode.Position, position / Constants.CLIMBER_METERS_PER_PULSE);
    }

    /**
     * Sets the climber motor's zero to its current position
     */
    public void setSensorZero() {
        climberMotor.setSelectedSensorPosition(0);
    }

    /**
     * Gets the climber motor's position in meters
     * 
     * @return the climber motor's position in meters
     */
    public double getClimberPos() {
        return climberMotor.getSelectedSensorPosition() * Constants.CLIMBER_METERS_PER_PULSE;
    }

    
    /**
     * Gets the bottom limit switch's current state
     * 
     * @return boolean representing the bottom limit's current state
     */
    public boolean getBottomLimit() {
        return !bottomLimit.get();
    }

    /**
     * Sets the position of the secondary climber arm based on enum value
     * 
     * @param solenoidPosition
     *                         <ul>
     *                         <li>kNeutral - Neither side is pressurized
     *                         <li>kUp - Raise the climber arm
     *                         <li>kDown - Lower the climber arm
     */
    public void setSolenoidPosition(SolenoidPosition solenoidPosition) {
        switch (solenoidPosition) {
            case kNeutral:
                secondaryArm.set(Value.kOff);
                break;
            case kUp:
                secondaryArm.set(Value.kReverse);
                break;
            case kDown:
                secondaryArm.set(Value.kForward);
                break;
        }
    }

    /**
     * sets the output of the climber motor to a specified percent
     * 
     * @param percentOutput the percent to set the climber motor to
     */
    public void setOutput(double percentOutput) {
        climberMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    /**
     * gets the current supply current to the motor
     * 
     * @return the supply current to the motor in amps
     */
    public double getClimberCurrent() {
        return climberMotor.getSupplyCurrent();
    }

    public double getArmAngle() {
        return (armEncoder.get() * Constants.CLIMBER_ARM_DEGREES_PER_PULSE) + Constants.CLIMBER_FLOPPY_ARM_OFFSET;
    }

    public double getArmSpeed() {
        return armEncoder.getRate();
    }

    public void resetArmAngle() {
        armEncoder.reset();
    }

    public void setPIDSlot(int slot) {
        climberMotor.selectProfileSlot(slot, slot);
    }

    public void setDebugMode(boolean debug) {
        this.debug = debug;
    }
}