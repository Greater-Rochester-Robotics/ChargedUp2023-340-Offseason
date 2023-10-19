// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.Positions;

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
            intake.setMotors(0.0, IntakeConstants.INNER_HOLD_SPEED),
            arm.setPosition(Positions.SAFE)
        );
    }
}
