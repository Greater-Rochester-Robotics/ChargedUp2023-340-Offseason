// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.ctre;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.subsystems.swervelib.interfaces.SwerveMoveMotor;

/** Add your docs here. */
public class SwerveMoveTalonFX implements SwerveMoveMotor {
    private TalonFX driveMotor;
    private final double ENC_TO_METERS_CONV_FACTOR;
    private final StatusSignal<Double> positionSignal;
    private final StatusSignal<Double> velocitySignal;

    public SwerveMoveTalonFX(int driveMotorID) {
        this(driveMotorID, new TalonFXConfiguration());
    }

    public SwerveMoveTalonFX(int driveMotorID, TalonFXConfiguration config) {
        this(driveMotorID, config, 0.0);
    }

    public SwerveMoveTalonFX(int driveMotorID, TalonFXConfiguration config, double encToMetersConvFactor) {
        driveMotor = new TalonFX(driveMotorID);
        config.CurrentLimits.SupplyCurrentLimit = Constants.MAXIMUM_VOLTAGE;
        driveMotor.getConfigurator().apply(config);

        ENC_TO_METERS_CONV_FACTOR = encToMetersConvFactor;

        positionSignal = driveMotor.getPosition();
        positionSignal.setUpdateFrequency(20);

        velocitySignal = driveMotor.getVelocity();
        velocitySignal.setUpdateFrequency(20);
    }
    
    public void setDriveDutyCycle(double dutyCycle){
        driveMotor.set(dutyCycle);
    }

    public void setDriveSpeed(double speed){
        VelocityVoltage request = new VelocityVoltage(speed /ENC_TO_METERS_CONV_FACTOR/10).withSlot(0);
        driveMotor.setControl(request);
    }

    /**
     * A method to set the drive motor to brake
     * @param brakeOn
     */
    public void setDriveMotorBrake(boolean brakeOn){
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = brakeOn ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(config);
    }

    public double getDriveDistance(){
        return positionSignal.getValue()*ENC_TO_METERS_CONV_FACTOR;
    }

    /**
     * A method to set the position of the drive encoder to zero,
     * essentially resetting it. 
     */
    public void resetDriveMotorEncoder(){
        driveMotor.setRotorPosition(0.0);
    }

    /**
     * Returns the speed of the drive wheel in Meters per second
     * 
     * @return speed of the drive wheel
     */
    public double getDriveVelocity(){
        return positionSignal.getValue()*ENC_TO_METERS_CONV_FACTOR*10;
    }

    /**
     * sets the drive motor's PIDF for the PIDF controller on the controller
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public void setDriveMotorPIDF(double P, double I, double D, double F){
        Slot0Configs config = new Slot0Configs();
        config.kP = P;
        config.kI = I;
        config.kD = D;
        config.kS = 0;
        config.kV = 0;
        driveMotor.getConfigurator().apply(config);
    }

    public void stopMotor(){
        driveMotor.set(0.0);
    }

    public void enableVoltageCompensation(double a) {}
}
