package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    private DigitalInput primaryArmSwitch;

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
        climberMotor.setInverted(false);
        climberMotor.setSensorPhase(false);
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.config_kP(0, Constants.CLIMBER_RAISE_P);
        climberMotor.config_kI(0, Constants.CLIMBER_RAISE_I);
        climberMotor.config_kD(0, Constants.CLIMBER_RAISE_D);
        climberMotor.config_kF(0, Constants.CLIMBER_RAISE_F);
        // climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, deviceID)


        /* Might need to change pneumatics module type */
        secondaryArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMBER_SOLENOID_DOWN,
                Constants.CLIMBER_SOLENOID_UP);

        bottomLimit = new DigitalInput(Constants.CLIMBER_BOTTOM_SWITCH);
        primaryArmSwitch = new DigitalInput(Constants.CLIMBER_PARM_SWITCH);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Bottom Climber Switch", getBottomLimit());
        SmartDashboard.putBoolean("Primary Climber Arm Switch", getPrimaryArmLimit());
    }

    public void homeClimber() {
        climberMotor.set(ControlMode.Position, 0);
    }

    public void setClimberPos(double position) {
        climberMotor.set(ControlMode.Position, position);
    }

    public void setSensorZero() {
        climberMotor.setSelectedSensorPosition(0);
    }

    public void getClimberPos() {
        climberMotor.getSelectedSensorPosition();
    }

    public boolean getBottomLimit() {
        return bottomLimit.get();
    }

    public boolean getPrimaryArmLimit() {
        return primaryArmSwitch.get();
    }

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

    public void setOutput(double percentOutput) {
        climberMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public double getClimberCurrent() {
        return climberMotor.getSupplyCurrent();
    }
}