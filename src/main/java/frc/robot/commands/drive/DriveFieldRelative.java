/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away 
 * from the driver. If this is true, then left and right 
 * on the stick will cause the robot to move to the 
 * driver's left and right, respectively. This command 
 * does not end of its own accord so it must be interupted 
 * to end.
 */
public class DriveFieldRelative extends CommandBase {
  private boolean isVeloMode;
  private PIDController rotController = new PIDController(5.0, 0.0, 0.0);

  /**
   * Creates a new DriveFieldCentric.
   */
  public DriveFieldRelative(boolean isVeloMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    this.isVeloMode = isVeloMode;
    rotController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double  awaySpeed = Robot.robotContainer.getRobotForward(isVeloMode, false);
    double lateralSpeed = Robot.robotContainer.getRobotLateral(isVeloMode, false);
    double rotSpeed = Robot.robotContainer.getRobotRotation(isVeloMode);

    if (Robot.robotContainer.driveAlign()) {
      rotSpeed = rotController.calculate(MathUtil.angleModulus(RobotContainer.swerveDrive.getGyroInRadYaw()), Math.PI);
    } else if(Robot.robotContainer.driveMod()){
     //if secondary sticks used, replace with secondary sticks witha slow factor
     awaySpeed = Robot.robotContainer.getRobotForward(isVeloMode, true);
     lateralSpeed = Robot.robotContainer.getRobotLateral(isVeloMode, true);
    }

    RobotContainer.swerveDrive.driveFieldRelative(
      awaySpeed,
      lateralSpeed,
      rotSpeed, 
      isVeloMode
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
