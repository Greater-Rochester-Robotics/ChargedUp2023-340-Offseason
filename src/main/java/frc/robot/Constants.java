// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swervelib.SwervePIDFConfig;
import frc.robot.subsystems.swervelib.rev.NEOConfig;

/** Add your docs here. */
public class Constants {
    /* Factors of PI */
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;
    public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);//I only make this once

    public static final double MAXIMUM_VOLTAGE = 12.0;

    public static class SwerveDriveConstants {
        /* Swerve Module Positions */
        public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(0.219075,0.219075);//These are in meters
        public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-0.219075,0.219075);
        public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-0.219075,-0.219075);
        public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(0.219075,-0.219075); 

        /* Swerve Module Drive Motor Constants */
                //This is the conversion from one full rotation of the motor to the distance that the robot travels    
        public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.12  * Math.PI / 6.54; //Colsen wheel .12 meters diameter 6.54:1 ratio
        public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
        public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle
        public static final double MOTOR_MAXIMUM_VELOCITY = 5.3;//4.33 5.19
         //TODO: get this
        public static final double MAX_ROBOT_ROT_VELOCITY = 3.5;

        // public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY / DISTANCE_TO_MODULE_0;
        public static final double MAXIMUM_VOLTAGE = 12.0;
        public static final double SWERVE_DRIVE_P_VALUE = 0.001; //0.035
        public static final double SWERVE_DRIVE_I_VALUE = 0.0;
        public static final double SWERVE_DRIVE_D_VALUE = 0.0; //25
        public static final double SWERVE_DRIVE_FF_VALUE = 1 / (MOTOR_MAXIMUM_VELOCITY);
        public static final SwervePIDFConfig MOVE_PIDF = new SwervePIDFConfig(SWERVE_DRIVE_P_VALUE, SWERVE_DRIVE_I_VALUE, SWERVE_DRIVE_D_VALUE, SWERVE_DRIVE_FF_VALUE);
        public static final NEOConfig MOVE_CONFIG = new NEOConfig(MOVE_PIDF, true, true, MAXIMUM_VOLTAGE);


        /* Swerve Module Rotation constants */
        public static final double ENC_TO_RAD_CONV_FACTOR = TWO_PI / 13.71;
        public static final double SWERVE_ROT_P_VALUE = 0.5;//.1;
        public static final double SWERVE_ROT_I_VALUE = 0.0;
        public static final double SWERVE_ROT_D_VALUE = 0.1; 
        public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
        public static final double SWERVE_ROT_FF_VALUE = 0.0;
        public static final SwervePIDFConfig ROTATE_PIDF = new SwervePIDFConfig(SWERVE_ROT_P_VALUE, SWERVE_ROT_I_VALUE, SWERVE_ROT_D_VALUE, SWERVE_ROT_FF_VALUE);
        public static final NEOConfig ROTATE_CONFIG = new NEOConfig(ROTATE_PIDF, true, false, MAXIMUM_VOLTAGE);
        public static final double SWERVE_MODULE_TOLERANCE = 0.1;
        public static final double ROTATIONAL_VELOCITY_TOLERANCE = 1.0;

        /* Robot Rotation PID controller constants */
        public static final double ROBOT_SPIN_PID_TOLERANCE = Math.toRadians(0.5);
        public static final double MINIMUM_ROTATIONAL_OUTPUT = 0.10;

        public static final double ROBOT_SPIN_P = 1.55;//tuned for drive/climber bot
        public static final double ROBOT_SPIN_I = 0.0;
        public static final double ROBOT_SPIN_D = 0.01;
    
        public static final double ROBOT_COUNTER_SPIN_P = 1.1;
        public static final double ROBOT_COUNTER_SPIN_I = 0.0;
        public static final double ROBOT_COUNTER_SPIN_D = 0.001;

        /* Path positional PID constants DriveFollowTrajectory */
        public static final double DRIVE_POS_ERROR_CONTROLLER_P = .33; // 10
        public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0.0005;
        public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0.0;//0.05;
        public static final double DRIVE_ROTATION_CONTROLLER_P = 1.50;
        public static final double DRIVE_ROTATION_CONTROLLER_I = 0.0;
        public static final double DRIVE_ROTATION_CONTROLLER_D = 0.01;
        // public static final double DRIVE_MAX_ANGULAR_VELOCITY = 13.5;//10.8;//PathFollowing
        // public static final double DRIVE_MAX_ANGULAR_ACCEL = 8.5;//7.03;//PathFollowing
        // public static final double DRIVE_ROTATION_MIN_VELOCITY = 25;
        public static final double PATH_MAXIMUM_VELOCITY = 4.0;
        public static final double PATH_MAXIMUM_ACCELERATION = 5.0;

        /* Aiming Values*/
        public static final Translation2d FIELD_CENTER = new Translation2d();

        /*Drive balance robot constants */
        public static final double DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE = 7;
        public static final double DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE = 3;
        public static final double DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE_AUTO = 13;
        public static final double DRIVE_BALANCE_ROBOT_MAX_SPEED = 0.10;
    }

    public static class ArmConstants {

        public static final double ABS_ENC_TO_RAD_CONVERSION_FACTOR = TWO_PI;

        public static final double ARM_SPEED_UP = 0.2;
        public static final double ARM_SPEED_DOWN = -0.2;
        
        public static final double ARM_P = 2.2;
        public static final double ARM_I = 0;
        public static final double ARM_D = 0;
        public static final double ARM_F = 0;

        public static final double ARM_MAX_PID_OUTPUT = .2;

        public static final double ARM_TOLERANCE = 0.05;

        public static final double ARM_MAX = 0.635;
        public static final double ARM_MIN = 0.43;

        public static final double ARM_MAX_MANUAL_DUTY_CYCLE = 0.3;
        public static final double WRIST_MAX_MANUAL_DUTY_CYCLE = 0.3;

        public static final double WRIST_SPEED_UP = 0.2;
        public static final double WRIST_SPEED_DOWN = -0.2;

        public static final double WRIST_P = 1.1;
        public static final double WRIST_I = 0;
        public static final double WRIST_D = 0.3;
        public static final double WRIST_F = 0;
        public static final double WRIST_GRAV_FF = 0.0;//.12;

        public static final double WRIST_MAX_PID_OUTPUT = .9;

        public static final double WRIST_TOLERANCE = 0.05;

        public static final double WRIST_MAX = 5.55;
        public static final double WRIST_MIN = 3.75;

        public static final class Positions {
            public static final Arm.Position INTAKE = new Arm.Position(.435, 3.8);
            public static final Arm.Position SAFE = new Arm.Position(.435, 5.5);
            public static final Arm.Position SHOOT_LOW = new Arm.Position(.435, 4.0);
            public static final Arm.Position SHOOT_MID = new Arm.Position(.435, 5.2);
            public static final Arm.Position SHOOT_HIGH = new Arm.Position(.435, 5.2);
            public static final Arm.Position SHOOT_FAR = new Arm.Position(.63, 4.5);
        }
    }
    public static class IntakeConstants {
        public static final double OUTER_INTAKE_SPEED = 0.2;
        public static final double INNER_INTAKE_SPEED = 0.8;
        public static final double INNER_HOLD_SPEED = 0.2;
        public static final double SHOOT_SPEED_LOW = -0.07;
        public static final double SHOOT_SPEED_MID = -0.16;
        public static final double SHOOT_SPEED_HIGH = -0.30;
        public static final double SHOOT_SPEED_FAR_UPPER = -0.9;
        public static final double SHOOT_SPEED_FAR_LOWER = -0.8;
        public static final double SHOOT_SPEED_INNER = -1.0;
    }

    public static class RobotMap {
        /* Rev Robotics SparkMAXs */
        public static final int FRONT_LEFT_MOVE_MOTOR = 2;//drive module 0
        public static final int FRONT_LEFT_ROTATE_MOTOR = 3;//drive module 0

        public static final int REAR_LEFT_MOVE_MOTOR = 4;//drive module 1
        public static final int REAR_LEFT_ROTATE_MOTOR = 5;//drive module 1

        public static final int REAR_RIGHT_MOVE_MOTOR = 6;//drive module 2
        public static final int REAR_RIGHT_ROTATE_MOTOR = 7;//drive module 2
        
        public static final int FRONT_RIGHT_MOVE_MOTOR = 8;//drive module 3
        public static final int FRONT_RIGHT_ROTATE_MOTOR = 9;//drive module 3

        public static final int ARM_MOTOR = 10; 
        public static final int WRIST_MOTOR = 11;

        public static final int WRIST_INNER_LIMIT_SWITCH = 1;
        public static final int WRIST_OUTER_LIMIT_SWITCH = 2;

        public static final int INTAKE_UPPER_MOTOR = 20;
        public static final int INTAKE_LOWER_MOTOR = 21;
        public static final int INTAKE_INNER_MOTOR = 22;
        
        public static final int INTAKE_CUBE_LIMIT_DIGITAL_INPUT = 0;

        /* ctre */
        public static final int FRONT_LEFT_CAN_CODER = 3;
        public static final int REAR_LEFT_CAN_CODER = 5;
        public static final int REAR_RIGHT_CAN_CODER = 7;
        public static final int FRONT_RIGHT_CAN_CODER = 9;
    }

    public static class ControllerConstants {
        public static final double CONTROLLER_DEADZONE = 0.15;
        public static final double DRIVER_SPEED_SCALE_LINEAR = 0.75;
        public static final double DRIVER_SPEED_SCALE_ROTATIONAL = .75;
        public static final double DRIVER_ROT_SPEED_SCALE_EXPONENTIAL = 2.0;
        public static final double DRIVER_PERCENT_ROT_SPEED_SCALE_LINEAR = 1.0;
    }
}
