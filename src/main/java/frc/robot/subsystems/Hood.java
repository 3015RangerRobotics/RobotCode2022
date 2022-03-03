// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private TalonSRX hoodMotor;
    private DigitalInput limitSwitch;
    public double pos;
    boolean hasBeenHomed = false;

    public Hood() {
        hoodMotor = new TalonSRX(Constants.HOOD_MOTOR);
        hoodMotor.config_kP(0, Constants.HOOD_CONTROLLER_P);
        hoodMotor.config_kI(0, Constants.HOOD_CONTROLLER_I);
        hoodMotor.config_kD(0, Constants.HOOD_CONTROLLER_D);
        // hoodMotor.getPIDController().setOutputRange(-1, 1);
        // hoodMotor.setInverted(true);
        // hoodMotor.getEncoder().setInverted(true);
        hoodMotor.setInverted(true);
        hoodMotor.configForwardSoftLimitThreshold(Constants.HOOD_MAX_ANGLE / Constants.HOOD_DEGREES_PER_PULSE);
        hoodMotor.configReverseSoftLimitThreshold(0);
        hoodMotor.configForwardSoftLimitEnable(true);
        hoodMotor.configReverseSoftLimitEnable(true);
        limitSwitch = new DigitalInput(Constants.HOOD_SWITCH_CHANNEL);
        enableForwardSoftLimit(false);
        setReverseLimit(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hood Limit Switch", getReverseLimit());
        SmartDashboard.putNumber("Hood Angle", getHoodPosition());
        SmartDashboard.putBoolean("Hood has been homed", hasBeenHomed);
    }

    public double getHoodPosition() {
        return hoodMotor.getSelectedSensorPosition() * Constants.HOOD_DEGREES_PER_PULSE;
    }

    public void setHoodPosition(double position) {
        hoodMotor.set(ControlMode.Position, position / Constants.HOOD_DEGREES_PER_PULSE);
    }

    public boolean getReverseLimit() {
        return limitSwitch.get();
    }

    public void setReverseLimit(boolean enable) {
        hoodMotor.configReverseSoftLimitEnable(enable);
    }

    public void setHoodOutputPercentage(double percentage) {
        // if (percentage < 0 && getReverseLimit()) {
        // hoodMotor.set(0);
        // } else {
        hoodMotor.set(ControlMode.PercentOutput, percentage);
        // }
    }

    public void enableForwardSoftLimit(boolean enabled) {
        hoodMotor.configForwardSoftLimitEnable(enabled);
    }

    public void resetZero() {
        hoodMotor.setSelectedSensorPosition(0);
    }

    public boolean isPrimed() {
	    return (Math.abs(pos - getHoodPosition()) <= Constants.HOOD_TOLERANCE);
    }

    public boolean hasBeenHomed() {
        return hasBeenHomed;
    }

    public void setHomed() {
        hasBeenHomed = true;
    }
}
