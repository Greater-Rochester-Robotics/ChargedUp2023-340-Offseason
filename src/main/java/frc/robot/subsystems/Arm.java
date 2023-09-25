// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private AbsoluteEncoder armEncoder;
  private SparkMaxPIDController armController;

  private double targetArmPosition = 0;
  private double targetWristPosition = 0;

  private CANSparkMax wristMotor;
  private AbsoluteEncoder wristEncoder;
  private SparkMaxPIDController wristController;

  private DigitalInput wristInnerLimitSwitch;
  private DigitalInput wristOuterLimitSwitch;
  
  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.RobotMap.ARM_MOTOR, MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    armController = armMotor.getPIDController();

    wristMotor = new CANSparkMax(Constants.RobotMap.WRIST_MOTOR, MotorType.kBrushless);
    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristController = wristMotor.getPIDController();

    // TODO: put correct settings
    // Arm motor settings.
    armMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    armMotor.setInverted(true);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setClosedLoopRampRate(1);

    // Arm frame settings.
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);


    // Arm encoder settings.
    armEncoder.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR);
    armEncoder.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR / 60);
    armEncoder.setInverted(true);
    //armEncoder.setZeroOffset(3.7332022);

    armController.setP(ArmConstants.armControllerP);
    armController.setI(ArmConstants.armControllerI);
    armController.setD(ArmConstants.armControllerD);
    armController.setFF(ArmConstants.armControllerFF);

     // Wrist motor settings.
     wristMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
     wristMotor.setInverted(true);
     wristMotor.setIdleMode(IdleMode.kBrake);
     wristMotor.setClosedLoopRampRate(1);
     
     // Wrist frame settings.
     wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
     wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
     wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
     wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
     wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
     wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
     wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);
 
     wristInnerLimitSwitch = new DigitalInput(Constants.RobotMap.WRIST_INNER_LIMIT_SWITCH);
     wristOuterLimitSwitch = new DigitalInput(Constants.RobotMap.WRIST_OUTER_LIMIT_SWITCH);

  }

  @Override
  public void periodic() {}

  // arm methods

  public void setArmDutyCycle(double speed){
    armController.setReference(speed, ControlType.kDutyCycle);
  }

  public void armToPosition(double position){
    armController.setReference(position, ControlType.kPosition);
    targetArmPosition = position;
  }

  public void armStop(){
    armController.setReference(0, ControlType.kDutyCycle);
  }

  public void armHoldPosition(){
    armController.setReference(getArmPosition(), ControlType.kPosition);
  }

  public double getArmPosition() {
    return armEncoder.getPosition();
  }

  public boolean hasArmReachedPosition(){
    return Math.abs(armEncoder.getPosition() - targetArmPosition) < ArmConstants.armTolerance;
  }

  // wrist methods

  public void setWristDutyCycle(double speed){
    wristController.setReference(speed, ControlType.kDutyCycle);
  }


  public void wristToPosition(double position){
    wristController.setReference(position, ControlType.kPosition);
    targetWristPosition = position;
  }

  public void wristStop(){
    wristController.setReference(0, ControlType.kDutyCycle);
  }

  public void wristHoldPosition(){
    wristController.setReference(getWristPosition(), ControlType.kPosition);
  } 

  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  public boolean hasWristReachedPosition(){
    return Math.abs(wristEncoder.getPosition() - targetWristPosition) < ArmConstants.wristTolerance;
  }

  public boolean getInnerWistLimitSwitch(){
    return wristInnerLimitSwitch.get();
  }

  public boolean getOuterWistLimitSwitch(){
    return wristOuterLimitSwitch.get();
  }

  // arm wrist methods

  public void armWristToPosition(double armPosition, double wristPosition){
    armToPosition(armPosition);
    wristToPosition(wristPosition);
  }

  public void armWristStop(){
    armStop();
    wristStop();
  }

  public boolean hasArmWristReachedPosition(){
    return hasArmReachedPosition() && hasWristReachedPosition();
  }

}
