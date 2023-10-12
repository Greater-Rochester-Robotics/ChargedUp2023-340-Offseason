// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmIntakeIntakeCube extends SequentialCommandGroup {
  /** Creates a new ArmIntakeIntakeCube. */
  public ArmIntakeIntakeCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.deadline(
        intake.getIntakeCubeCommand(),
        new ArmWristToPosition(ArmConstants.INTAKE_ARM_POSITION, ArmConstants.INTAKE_WIRIST_POSITON)
      ),
      new ArmWristToPosition(ArmConstants.SAFE_ARM_POSITION, ArmConstants.SAFE_WRIST_POSITION)
    );
  }
}