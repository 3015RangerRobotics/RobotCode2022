// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {
    private CANSparkMax hoodMotor;
    private SparkMaxLimitSwitch reverseLimitSwitch;
    private double setPos = 0;
    private Timer timer = new Timer();

    private enum State {
        kSetPosition,
        kNeutral,
        kHoming
    }

    public State state = State.kNeutral;

    public Hood() {
        hoodMotor = new CANSparkMax(Constants.HOOD_MOTOR, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.getEncoder().setPositionConversionFactor(Constants.HOOD_DEGREES_PER_ROTATION);
        hoodMotor.getPIDController().setP(Constants.HOOD_CONTROLLER_P);
        hoodMotor.getPIDController().setI(Constants.HOOD_CONTROLLER_I);
        hoodMotor.getPIDController().setD(Constants.HOOD_CONTROLLER_D);
        hoodMotor.getPIDController().setOutputRange(-1, 1);
        // hoodMotor.setInverted(true);
        // hoodMotor.getEncoder().setInverted(true);
        hoodMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.HOOD_MAX_ANGLE);
        hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        reverseLimitSwitch = hoodMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        enableForwardSoftLimit(true);
    }

    @Override
    public void periodic() {
    }

    public double getHoodPosition() {
        return hoodMotor.getEncoder().getPosition();
    }

    public void setHoodPosition(double position) {
        hoodMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    public boolean getReverseLimit() {
        return hoodMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public void setHoodOutputPercentage(double percentage) {
        hoodMotor.set(percentage);
    }

    public void enableForwardSoftLimit(boolean enabled) {
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, enabled);
    }

    public void resetZero() {
        hoodMotor.getEncoder().setPosition(0);
    }
}
