// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private TalonSRX hoodMotor;
    private DigitalInput limitSwitch;
    public double pos;
    boolean hasBeenHomed = false;
    boolean doRestPosition = true;
    boolean debug = false;
    Timer timer;

    public Hood() {
        hoodMotor = new TalonSRX(Constants.HOOD_MOTOR);
        hoodMotor.config_kP(0, Constants.HOOD_CONTROLLER_P);
        hoodMotor.config_kI(0, Constants.HOOD_CONTROLLER_I);
        hoodMotor.config_kD(0, Constants.HOOD_CONTROLLER_D);
        // hoodMotor.getPIDController().setOutputRange(-1, 1);
        // hoodMotor.setInverted(true);
        // hoodMotor.getEncoder().setInverted(true);
        hoodMotor.setInverted(true);
        hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 251);
        hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 247);
        hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 239);
        hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
        hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 229);
        hoodMotor.configForwardSoftLimitThreshold(Constants.HOOD_MAX_ANGLE / Constants.HOOD_DEGREES_PER_PULSE);
        hoodMotor.configReverseSoftLimitThreshold(0);
        hoodMotor.configForwardSoftLimitEnable(true);
        hoodMotor.configReverseSoftLimitEnable(true);
        limitSwitch = new DigitalInput(Constants.HOOD_SWITCH_CHANNEL);
        enableForwardSoftLimit(false);
        setReverseLimit(true);
        timer = new Timer();
        timer.start();
    }

    @Override
    public void periodic() {
        if (debug) {
            SmartDashboard.putBoolean("Hood Limit Switch", getReverseLimit());
            SmartDashboard.putNumber("Hood Angle", getHoodPosition());
            SmartDashboard.putBoolean("Hood has been homed", hasBeenHomed);
        }
        if (timer.hasElapsed(1) && hasBeenHomed && doRestPosition) {
            setHoodPosition(Constants.HOOD_REST_POSITION);
        }
    }

    public double getHoodPosition() {
        return hoodMotor.getSelectedSensorPosition() * Constants.HOOD_DEGREES_PER_PULSE;
    }

    public void setHoodPosition(double position) {
        if (!hasBeenHomed) {
            return;
        }
        hoodMotor.set(ControlMode.Position, position / Constants.HOOD_DEGREES_PER_PULSE);
        timer.reset();
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

    public void overriderRestPosition(boolean override) {
        this.doRestPosition = !override;
    }

    public void setDebugMode(boolean debug) {
        this.debug = debug;
    }
}
