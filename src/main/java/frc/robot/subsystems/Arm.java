// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  
  private AbsoluteEncoder armEncoder;

  private double targetPosition;

  private boolean isGoingToPosition = false;
  
  /** Creates a new Arm. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.RobotMap.ARM_MOTOR, MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);

  }

  @Override
  public void periodic() {

    if(isGoingToPosition){

      double currPosition = armEncoder.getPosition();

      if(Math.abs(currPosition - targetPosition) < Constants.ArmConstants.POSITION_TOLERANCE){
        //arm is within the set tolerance
        isGoingToPosition = false;

      } else if(currPosition < targetPosition){
        //arm should lift up
        setArmDutyCycle(Constants.ArmConstants.ARM_SPEED_UP);

      } else {
        //arm should go down
        setArmDutyCycle(Constants.ArmConstants.ARM_SPEED_DOWN);
      }

    }

  }

  private void setArmDutyCycle(double speed){
    armMotor.set(speed);
  }

  public void setArmSpeed(double speed){
    //unfinished
    
  }

  public void armToPosition(double position){
    targetPosition = position;
    isGoingToPosition = true;
  }

}
