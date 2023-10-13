// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.ctre;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.swervelib.interfaces.SwerveRotationMotor;

/** Add your docs here. */
public class SwerveRotationTalonFX implements SwerveRotationMotor {
    private TalonFX rotationMotor;
    private final StatusSignal<Double> positionSignal;
    private final StatusSignal<Double> velocitySignal;
    private final double ENC_TO_RADS_CONV_FACTOR;
    
    public SwerveRotationTalonFX(int rotationMotorID) {
        this(rotationMotorID, new TalonFXConfiguration());
    }

    public SwerveRotationTalonFX(int rotationMotorID, TalonFXConfiguration config) {
        this(rotationMotorID, config, 0.0);
    }

    public SwerveRotationTalonFX(int rotationMotorID, TalonFXConfiguration config, double encToRadConvFactor) {
        rotationMotor = new TalonFX(rotationMotorID);
        rotationMotor.getConfigurator().apply(config);
        ENC_TO_RADS_CONV_FACTOR = encToRadConvFactor;

        positionSignal = rotationMotor.getPosition();
        positionSignal.setUpdateFrequency(20);

        velocitySignal = rotationMotor.getVelocity();
        velocitySignal.setUpdateFrequency(20);
    }

    @Override
    public void setRotationMotorPIDF(double P, double I, double D, double F) {
        Slot0Configs config = new Slot0Configs();
        config.kP = P;
        config.kI = I;
        config.kD = D;
        config.kS = 0;
        config.kV = 0;
        rotationMotor.getConfigurator().apply(config);
    }

    @Override
    public double getRelEncCount() {
        return positionSignal.getValue()*ENC_TO_RADS_CONV_FACTOR;
    }

    @Override
    public double getRelEncSpeed() {
        return velocitySignal.getValue()*ENC_TO_RADS_CONV_FACTOR*10;
    }

    @Override
    public void driveRotateMotor(double dutyCycle) {
        rotationMotor.set(dutyCycle);
    }

    @Override
    public void setRotationMotorPosition(double output) {
        PositionVoltage request = new PositionVoltage(output /ENC_TO_RADS_CONV_FACTOR/10).withSlot(0);
        rotationMotor.setControl(request);
    }

    @Override
    public void stopRotation() {
        rotationMotor.set(0.0);
    }
    
}
