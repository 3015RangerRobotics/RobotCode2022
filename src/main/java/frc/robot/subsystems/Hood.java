// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private CANSparkMax hoodMotor;
    private DigitalInput limitSwitch;
    public double pos;

    public Hood() {
        hoodMotor = new CANSparkMax(Constants.HOOD_MOTOR, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.getPIDController().setP(Constants.HOOD_CONTROLLER_P);
        hoodMotor.getPIDController().setI(Constants.HOOD_CONTROLLER_I);
        hoodMotor.getPIDController().setD(Constants.HOOD_CONTROLLER_D);
        // hoodMotor.getPIDController().setOutputRange(-1, 1);
        // hoodMotor.setInverted(true);
        // hoodMotor.getEncoder().setInverted(true);
        hoodMotor.setSoftLimit(SoftLimitDirection.kForward,
                (float) (Constants.HOOD_MAX_ANGLE / Constants.HOOD_DEGREES_PER_ROTATION));
        hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        limitSwitch = new DigitalInput(Constants.HOOD_SWITCH_CHANNEL);
        enableForwardSoftLimit(true);
        setReverseLimit(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hood Limit Switch", getReverseLimit());
        SmartDashboard.putNumber("Hood Angle", getHoodPosition());
    }

    public double getHoodPosition() {
        return hoodMotor.getEncoder().getPosition() * Constants.HOOD_DEGREES_PER_ROTATION;
    }

    public void setHoodPosition(double position) {
        hoodMotor.getPIDController().setReference(position / Constants.HOOD_DEGREES_PER_ROTATION, ControlType.kPosition);
    }

    public boolean getReverseLimit() {
        return limitSwitch.get();
    }

    public void setReverseLimit(boolean enable) {
        hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    public void setHoodOutputPercentage(double percentage) {
        // if (percentage < 0 && getReverseLimit()) {
        // hoodMotor.set(0);
        // } else {
        hoodMotor.set(percentage);
        // }
    }

    public void enableForwardSoftLimit(boolean enabled) {
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, enabled);
    }

    public void resetZero() {
        hoodMotor.getEncoder().setPosition(0);
    }

    public boolean isPrimed() {
	    return (Math.abs(pos - getHoodPosition()) <= Constants.HOOD_TOLERANCE);
    }
}
