package org.team340.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.SPI;
import org.team340.lib.drivers.ADIS16470.CalibrationTime;
import org.team340.lib.drivers.ADIS16470.IMUAxis;
import org.team340.lib.math.Math2;
import org.team340.lib.swerve.SwerveBase.SwerveMotorType;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.robot.subsystems.Arm;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double PERIOD = 0.020;
    public static final double VOLTAGE = 12.0;
    public static final double FIELD_LENGTH = 16.5417;
    public static final double FIELD_WIDTH = 8.0136;

    /**
     * Driver and co-driver controller constants.
     */
    public static final class ControllerConstants {

        public static final int DRIVER = 0;

        public static final double JOYSTICK_DEADBAND = 0.1;
        public static final double JOYSTICK_THRESHOLD = 0.7;
        public static final double TRIGGER_DEADBAND = 0.1;
        public static final double TRIGGER_THRESHOLD = 0.1;

        public static final double DRIVE_EXP = 1.0;
        public static final double DRIVE_MULTIPLIER = 0.3;
        public static final double DRIVE_MULTIPLIER_MODIFIED = 0.6;

        public static final double DRIVE_ROT_EXP = 2.0;
        public static final double DRIVE_ROT_MULTIPLIER = 0.45;
    }

    /**
     * Map of hardware device IDs.
     */
    public static final class RobotMap {

        public static final class CANBus {

            public static final int FRONT_LEFT_MOVE = 2;
            public static final int FRONT_LEFT_TURN = 3;
            public static final int BACK_LEFT_MOVE = 4;
            public static final int BACK_LEFT_TURN = 5;
            public static final int BACK_RIGHT_MOVE = 6;
            public static final int BACK_RIGHT_TURN = 7;
            public static final int FRONT_RIGHT_MOVE = 8;
            public static final int FRONT_RIGHT_TURN = 9;
        }

        public static final int ARM_MOTOR = 10;
        public static final int WRIST_MOTOR = 11;

        public static final int WRIST_INNER_LIMIT_SWITCH = 1;
        public static final int WRIST_OUTER_LIMIT_SWITCH = 2;

        public static final int INTAKE_UPPER_MOTOR = 20;
        public static final int INTAKE_LOWER_MOTOR = 21;
        public static final int INTAKE_INNER_MOTOR = 22;

        public static final int INTAKE_CUBE_LIMIT_DIGITAL_INPUT = 0;
    }

    public static class ArmConstants {

        public static final double ABS_ENC_TO_RAD_CONVERSION_FACTOR = Math2.TWO_PI;

        public static final double ARM_SPEED_UP = 0.2;
        public static final double ARM_SPEED_DOWN = -0.2;

        public static final double ARM_P = 2.2;
        public static final double ARM_I = 0;
        public static final double ARM_D = 0;
        public static final double ARM_F = 0;

        public static final double ARM_MAX_PID_OUTPUT = .2;

        public static final double ARM_TOLERANCE = 0.05;

        public static final double ARM_MAX = 1.0;
        public static final double ARM_MIN = 0.43;

        public static final double ARM_MAX_MANUAL_DUTY_CYCLE = 0.3;
        public static final double WRIST_MAX_MANUAL_DUTY_CYCLE = 0.3;

        public static final double WRIST_SPEED_UP = 0.2;
        public static final double WRIST_SPEED_DOWN = -0.2;

        public static final double WRIST_P = 1.1;
        public static final double WRIST_I = 0;
        public static final double WRIST_D = 0.3;
        public static final double WRIST_F = 0;
        public static final double WRIST_GRAV_FF = 0.0; //.12;

        public static final double WRIST_MAX_PID_OUTPUT = .9;

        public static final double WRIST_TOLERANCE = 0.05;

        public static final double WRIST_MAX = 3.1;
        public static final double WRIST_MIN = 0.55;

        public static final double WRIST_RAMP_RATE = 0.8;

        public static final class Positions {

            public static final Arm.Position INTAKE = new Arm.Position(.435, 0.61);
            public static final Arm.Position SAFE = new Arm.Position(.435, 3.0);
            public static final Arm.Position SHOOT_SLOW = new Arm.Position(.8, 1.65);
            public static final Arm.Position SHOOT_NORMAL = new Arm.Position(.8, 1.65);
            public static final Arm.Position SHOOT_FAST = new Arm.Position(1.0, 1.7);
        }
    }

    public static class IntakeConstants {

        public static final double OUTER_INTAKE_SPEED = 0.25;
        public static final double INNER_INTAKE_SPEED = 0.9;
        public static final double INNER_HOLD_SPEED = 0.1;
        public static final double SHOOT_SPEED_SLOW = -0.2;
        public static final double SHOOT_SPEED_NORMAL = -0.35;
        public static final double SHOOT_SPEED_FAST = -0.6;
        public static final double SHOOT_SPEED_INNER = -1.0;
    }

    public static final class SwerveConstants {

        private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
            .setLabel("Front Left")
            .useCANcoder(10, -3.8534, false)
            .setPosition(0.219075, 0.219075)
            .setMoveMotor(RobotMap.CANBus.FRONT_LEFT_MOVE, true, false)
            .setTurnMotor(RobotMap.CANBus.FRONT_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
            .setLabel("Back Left")
            .useCANcoder(11, -3.1124, false)
            .setPosition(-0.219075, 0.219075)
            .setMoveMotor(RobotMap.CANBus.BACK_LEFT_MOVE, true, false)
            .setTurnMotor(RobotMap.CANBus.BACK_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
            .setLabel("Back Right")
            .useCANcoder(12, -6.2234, false)
            .setPosition(-0.219075, -0.219075)
            .setMoveMotor(RobotMap.CANBus.BACK_RIGHT_MOVE, true, false)
            .setTurnMotor(RobotMap.CANBus.BACK_RIGHT_TURN, false, true);

        private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
            .setLabel("Front Right")
            .useCANcoder(13, -3.0265, false)
            .setPosition(0.219075, -0.219075)
            .setMoveMotor(RobotMap.CANBus.FRONT_RIGHT_MOVE, true, false)
            .setTurnMotor(RobotMap.CANBus.FRONT_RIGHT_TURN, false, true);

        public static final SwerveConfig CONFIG = new SwerveConfig()
            .useADIS16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, SPI.Port.kOnboardCS0, CalibrationTime._4s)
            .setPeriod(PERIOD)
            .setMovePID(0.001, 0.0, 0.0)
            .setMoveFF(0.0, 2.35, 0.0)
            .setTurnPID(0.5, 0.0, 15.0)
            .setRampRate(0.03, 0.03)
            .setPowerProperties(VOLTAGE, 40.0, 30.0)
            .setMechanicalProperties(7.5, 10.0, 4.0)
            .setSpeedConstraints(5.3, 7.0, 7.0, 25.0)
            .setMotorTypes(SwerveMotorType.SPARK_MAX_BRUSHLESS, SwerveMotorType.SPARK_MAX_BRUSHLESS)
            .setDiscretizationLookahead(0.040)
            .setStandardDeviations(0.1, 0.1, 0.1)
            .setFieldSize(FIELD_LENGTH, FIELD_WIDTH)
            .addModule(FRONT_LEFT)
            .addModule(BACK_LEFT)
            .addModule(BACK_RIGHT)
            .addModule(FRONT_RIGHT);

        public static final double POSE_ROT_P = 7.0;
        public static final double POSE_ROT_I = 0.0;
        public static final double POSE_ROT_D = 0.5;
        public static final Constraints POSE_ROT_CONSTRAINTS = new Constraints(6.0, 12.5);
    }
}
