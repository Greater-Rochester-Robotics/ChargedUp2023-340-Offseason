// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotContainer.*;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.Positions;
import frc.robot.subsystems.Arm.Position;

/** Add your docs here. */
public class Routines {
    private Routines() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
    
    public static Command intake(boolean waitForSensor) {
        if(waitForSensor) {
            return sequence(
                deadline(
                    intake.pickUpCube(waitForSensor),
                    arm.setPosition(Positions.INTAKE)
                ),
                storeCube()
            );
        } else { 
            return parallel(
                intake.pickUpCube(waitForSensor),
                arm.setPosition(Positions.INTAKE)
            );
        }
    }

    public static Command intake() {
        return intake(false);
    }

    public static Command storeCube() {
        return sequence(
            intake.setMotors(0.0),
            arm.setPosition(Positions.SAFE)
        );
    }

    public static Command shoot(Position position, double upperSpeed, double lowerSpeed) {
        return sequence(
            arm.setPosition(position),
            parallel(
                intake.setMotors(upperSpeed, lowerSpeed),
                arm.holdPosition()
            )
        );
    }

    public static Command shoot(Position position, double speed) {
        return shoot(position, speed, speed);
    }
    
    public static Command shootLow() {
        return shoot(Positions.SHOOT_LOW, IntakeConstants.SHOOT_SPEED_LOW);
    }
    
    public static Command shootMid() {
        return shoot(Positions.SHOOT_MID, IntakeConstants.SHOOT_SPEED_MID);
    }
    
    public static Command shootHigh() {
        return shoot(Positions.SHOOT_HIGH, IntakeConstants.SHOOT_SPEED_HIGH);
    }

    public static Command shootFar() {
        return shoot(Positions.SHOOT_FAR, IntakeConstants.SHOOT_SPEED_FAR, IntakeConstants.SHOOT_SPEED_FAR + 0.1);
    }
}
