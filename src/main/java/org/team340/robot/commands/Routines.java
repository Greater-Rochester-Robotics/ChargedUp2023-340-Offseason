package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.team340.robot.Constants.ArmConstants.Positions;
import org.team340.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class Routines {

    private Routines() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command intake() {
        return intake(false);
    }

    public static Command intake(boolean waitForSensor) {
        if (waitForSensor) {
            return sequence(deadline(intake.pickUpCube(waitForSensor), arm.setPosition(Positions.INTAKE)), storeCube());
        } else {
            return parallel(intake.pickUpCube(waitForSensor), arm.setPosition(Positions.INTAKE));
        }
    }

    public static Command storeCube() {
        return sequence(intake.setMotors(0.0, IntakeConstants.INNER_HOLD_SPEED), arm.setPosition(Positions.SAFE));
    }
}
