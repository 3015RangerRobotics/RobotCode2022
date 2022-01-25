package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor;
    private DoubleSolenoid secondaryArm;
    private DigitalInput bottomLimit;
    private DigitalInput topLimit;
    private DigitalInput primaryArmSwitch;
    private DigitalInput secondaryArmSwitch;

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

    /**
     * <ul>
     * <li>kDefault - default state of the climber, does not move up or down
     * <li>kHoming - moves the climber to the bottom limit switch and resets the
     * home value
     * <li>kRaise - raises the climber
     * <li>kLower - lowers the climber
     */
    public enum State {
        kDefault,
        kHoming,
        kExtend,
        kRetract
    }

    State state = State.kDefault;

    public Climber() {
        climberMotor = new TalonFX(Constants.CLIMBER_MOTOR);
        climberMotor.configVoltageCompSaturation(12.5);
        climberMotor.enableVoltageCompensation(true);
        climberMotor.setInverted(false);
        climberMotor.setSensorPhase(false);
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.config_kP(0, Constants.CLIMBER_RAISE_P);
        climberMotor.config_kI(0, Constants.CLIMBER_RAISE_I);
        climberMotor.config_kD(0, Constants.CLIMBER_RAISE_D);
        climberMotor.config_kF(0, Constants.CLIMBER_RAISE_F);
        climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, normalOpenOrClose, deviceID)

        /* Might need to change pneumatics module type */
        secondaryArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_DOWN,
                Constants.CLIMBER_SOLENOID_UP);

        bottomLimit = new DigitalInput(Constants.CLIMBER_BOTTOM_SWITCH);
        topLimit = new DigitalInput(Constants.CLIMBER_TOP_SWITCH);
        primaryArmSwitch = new DigitalInput(Constants.CLIMBER_PARM_SWITCH);
        secondaryArmSwitch = new DigitalInput(Constants.CLIMBER_SARM_SWITCH);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Bottom Climber Switch", getBottomLimit());
        SmartDashboard.putBoolean("Top Climber Switch", getTopLimit());
        SmartDashboard.putBoolean("Primary Climber Arm Switch", getPrimaryArmLimit());
        SmartDashboard.putBoolean("Secondary Climber Arm Switch", getSecondaryArmLimit());

        // switch (state) {
        //     case kHoming:
        //         if (getBottomLimit()) {
                    // climberMotor.set(ControlMode.Velocity, 0);
        //             climberMotor.setSelectedSensorPosition(0);
        //             state = State.kDefault;
        //         }
        //         climberMotor.set(ControlMode.Velocity, -0.25);
        //         break;
        //     case kExtend:
        //         if (getTopLimit()) {
        //             climberMotor.set(ControlMode.Velocity, 0);
        //             state = State.kDefault;
        //         }
        //         climberMotor.set(ControlMode.Velocity, 1);
        //         break;
        //     case kRetract:
        //         if (getBottomLimit()) {
        //             climberMotor.set(ControlMode.Velocity, 0);
        //             state = State.kDefault;
        //         }
        //         climberMotor.set(ControlMode.Velocity, -1);
        //     case kDefault:
        //         break;
        // }
    }

    public void homeClimber()
    {
        climberMotor.set(ControlMode.Position, 0);
    }

    public void setClimberPos(double position)
    {
        climberMotor.set(ControlMode.Position, position);
    }

    public void setSensorZero()
    {
        climberMotor.setSelectedSensorPosition(0);
    }

    public void getClimberPos()
    {
        climberMotor.getSelectedSensorPosition();
    }

    public boolean getBottomLimit() {
        return bottomLimit.get();
    }

    public boolean getTopLimit() {
        return topLimit.get();
    }

    public boolean getPrimaryArmLimit() {
        return primaryArmSwitch.get();
    }

    public boolean getSecondaryArmLimit() {
        return secondaryArmSwitch.get();
    }

    public void setSolenoidPosition(SolenoidPosition solenoidPosition) {
        switch (solenoidPosition) {
            case kNeutral:
                secondaryArm.set(Value.kOff);
                break;
            case kUp:
                secondaryArm.set(Value.kForward);
                break;
            case kDown:
                secondaryArm.set(Value.kReverse);
                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }
}