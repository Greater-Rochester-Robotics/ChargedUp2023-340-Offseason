// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.Positions;

public class ScoreTarget extends SubsystemBase {
  public enum Level {
    LOW,
    MID,
    HIGH,
    FAR
  }

  private Level level = Level.FAR;

  public Command setLevel(Level level) {
    return runOnce(() -> this.level = level);
  }

  public Command getArmCommand() {
    return new ProxyCommand(() -> {
      switch(level) {
        case LOW: return arm.setPosition(Positions.SHOOT_LOW);
        case MID: return arm.setPosition(Positions.SHOOT_MID);
        case HIGH: return arm.setPosition(Positions.SHOOT_HIGH);
        case FAR: return arm.setPosition(Positions.SHOOT_FAR);
        default: return Commands.none();
      }
    }) {{
      addRequirements(arm);
    }};
  }

  public Command getShootCommand() {
    return new ProxyCommand(() -> {
      switch(level) {
        case LOW: return intake.setMotors(IntakeConstants.SHOOT_SPEED_LOW);
        case MID: return intake.setMotors(IntakeConstants.SHOOT_SPEED_MID);
        case HIGH: return intake.setMotors(IntakeConstants.SHOOT_SPEED_HIGH);
        case FAR: return intake.setMotors(IntakeConstants.SHOOT_SPEED_FAR, IntakeConstants.SHOOT_SPEED_FAR + 0.1);
        default: return Commands.none();
      }
    }) {{
      addRequirements(intake);
    }};
  }
}
