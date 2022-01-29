// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {
    private CANSparkMax hoodMotor;
    private double setPos = 0;
    // private boolean holdAutoPos = false;
    private Timer timer = new Timer();

    private enum State {
        kSetPosition,
        // kAutoPosition,
        kNeutral,
        kHoming
    }
    public State state = State.kNeutral;

    public Hood(int id) {
        hoodMotor = new CANSparkMax(Constants.HOOD_MOTORS[id], MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.getEncoder().setPositionConversionFactor(Constants.HOOD_DEGREES_PER_ROTATION);
        hoodMotor.getPIDController().setP(Constants.HOOD_CONTROLLER_P);
        hoodMotor.getPIDController().setI(Constants.HOOD_CONTROLLER_I);
        hoodMotor.getPIDController().setD(Constants.HOOD_CONTROLLER_D);
        hoodMotor.getPIDController().setOutputRange(-1, 1);
        //       hoodMotor.setInverted(true);
        //       hoodMotor.getEncoder().setInverted(true);
        hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
        hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        enableForwardSoftLimit(true);
        //TODO: set up reverse limit switch(this is not a softlimit)
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        switch (state){
            case kSetPosition:
                setHoodPosition(setPos);
//                System.out.println(getHoodPosition());
                break;
            // case kAutoPosition:
            //     if(!holdAutoPos)
            //         setPos = getAutoPosition();
            //     setHoodPosition(setPos);
            //     break;
            case kHoming:
                //todo: use the reverse limit switch to home.
                setHoodOutputPercentage(-0.05);
//                System.out.println("howdy");
                if(Math.abs(hoodMotor.getEncoder().getVelocity() * 60) < 1 && timer.hasElapsed(0.25)){  //?
                    setStateNeutral();
                    enableForwardSoftLimit(true);
                    timer.stop();
                    hoodMotor.getEncoder().setPosition(0);
                }
                break;
            case kNeutral:
            default:
//                System.out.println("hello?");
                setHoodOutputPercentage(0);
                break;
        }
        //SmartDashboard.putNumber("Hood Position", getHoodPosition());
    }

    public void setStatePosition(double pos){
        setPos = pos;
        state = State.kSetPosition;
    }

    // public void setHoldAutoPos(boolean hold){
    //     this.holdAutoPos = hold;
    // }

    // public void setStateAutoPosition(){
    //     state = State.kAutoPosition;
    // }

    public void setStateHoming(){
        state = State.kHoming;
        timer.reset();
        timer.start();
    }

    public void setStateNeutral(){
        state = State.kNeutral;
    }

    public double getHoodPosition() {
        return hoodMotor.getEncoder().getPosition();
    }

    public void setHoodPosition(double position) {
        hoodMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    // public double getAutoPosition() {
    //     double d = RobotContainer.limelight.getAvgDistance();
    //     return Constants.HOOD_AUTO_POSITION_TABLE.lookup(d);
    // }

    // public boolean getReverseLimit() {
    //     return hoodMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    // }

    public void setHoodOutputPercentage(double percentage){
        hoodMotor.set(percentage);
    }

    public void enableForwardSoftLimit(boolean enabled){
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, enabled);
    }
}
