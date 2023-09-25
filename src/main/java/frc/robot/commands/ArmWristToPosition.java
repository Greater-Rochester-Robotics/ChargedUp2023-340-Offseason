// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmWristToPosition extends CommandBase {

  private double armPosition;
  private double wristPosition;

  public ArmWristToPosition(double armPosition, double wristPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
    this.armPosition = armPosition;
    this.wristPosition = wristPosition;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.arm.armWristToPosition(armPosition, wristPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.armWristStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.arm.hasArmWristReachedPosition();
  }
}
