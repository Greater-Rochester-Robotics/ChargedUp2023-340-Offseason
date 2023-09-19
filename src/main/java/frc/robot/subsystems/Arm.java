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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  
  private AbsoluteEncoder armEncoder;

  private SparkMaxPIDController armController;

  private boolean isMoving = false;

  private double targetPosition = 0;
  
  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.RobotMap.ARM_MOTOR, MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

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

  }

  @Override
  public void periodic() {

    if (isMoving && Math.abs(armEncoder.getPosition() - targetPosition) < ArmConstants.armTolerance) {
      isMoving = false;
      armStop();
    }

  }

  public void setArmDutyCycle(double speed){
    armController.setReference(speed, ControlType.kDutyCycle);
    isMoving = true;
  }

  public void armToPosition(double position){
    armController.setReference(position, ControlType.kPosition);
    targetPosition = position;
    isMoving = true;
  }

  public void armStop(){
    armController.setReference(0, ControlType.kDutyCycle);
    isMoving = false;
  }

  public void armHoldPosition(){
    armController.setReference(getPosition(), ControlType.kPosition);
    isMoving = false;
  }

  public boolean isArmMoving(){
    return isMoving;
  }

  public double getPosition() {
    return armEncoder.getPosition();
  }

}
