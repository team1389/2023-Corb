package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * define Hardware Ports in here
 * also swerve constants
 */
public class RobotMap {

    // Constants that are the same for the 4 swerve modules
    public static final class ModuleConstants {
        // Note: these are for the drive and turning motors
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
        public static final double DRIVE_GEAR_RATIO = 1 / 4.71; // 14T pinion
        public static final double TURN_GEAR_RATIO = 1 / (1); // this should be 1 this is correct

        public static final double FREE_MOTOR_SPEED_RPS = 5676 / 60; // RPM/60
        public static final double DRIVE_FREE_MAX_SPEED_MPS = (FREE_MOTOR_SPEED_RPS * WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVE_GEAR_RATIO;
        public static final double DRIVE_ROTATIONS_TO_METERS = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ROTATIONS_TO_RAD = TURN_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_RPM_TO_METERS_PER_SEC = DRIVE_ROTATIONS_TO_METERS / 60;
        public static final double TURNING_RPM_TO_RAD_PER_SEC = TURNING_ROTATIONS_TO_RAD / 60;
        public static final double P_TURNING = 0.75; // PID constant
        public static final double I_TURNING = 0.000001; // PID Constant
        public static final double D_TURNING = 0.00025; // PID constant
        public static final double P_DRIVE = 0.5; // PID constant

        public static final int DRIVE_CURRENT_LIMIT = 50; // amps
        public static final int TURN_CURRENT_LIMIT = 20;

        public static final double FL_ANGLE_OFFSET = Math.PI / 2; // CORRECT
        public static final double FR_ANGLE_OFFSET = Math.PI; // CORRECt
        public static final double BL_ANGLE_OFFSET = 0;
        public static final double BR_ANGLE_OFFSET = -Math.PI / 2;

    }

    // All the overall constants for the drivetrain
    public static final class DriveConstants {

        // Distance between right and left wheels (meters)
        public static final double ROBOT_WIDTH = 0.5588;

        // Distance between front and back wheels (meters)
        public static final double ROBOT_LENGTH = 0.5842;

        // Note positive x is forward
        // Wheel order: FR, FL, BR, BL
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2));

        // FL is front left, BR is back right, etc.
        public static final int ROLLER_MOTOR_PORT = 12;

        public static final int FL_DRIVE_PORT = 9;
        public static final int BL_DRIVE_PORT = 3;
        public static final int FR_DRIVE_PORT = 6;
        public static final int BR_DRIVE_PORT = 5;

        public static final int FL_TURN_PORT = 8;
        public static final int BL_TURN_PORT = 2;
        public static final int FR_TURN_PORT = 7;
        public static final int BR_TURN_PORT = 4;

        public static final int WRIST_MOTOR = 14; 

        // Sometimes encoders are mounted backwards based on robot design, this fixes
        // that although it's not a thing on stargazer
        public static final boolean FL_TURN_REVERSED = false;
        public static final boolean BL_TURN_REVERSED = false;
        public static final boolean FR_TURN_REVERSED = false;
        public static final boolean BR_TURN_REVERSED = false;

        public static final boolean FL_DRIVE_REVERSED = true;
        public static final boolean BL_DRIVE_REVERSED = true;
        public static final boolean FR_DRIVE_REVERSED = true;
        public static final boolean BR_DRIVE_REVERSED = true;

        public static final boolean FL_ABS_REVERSED = true;
        public static final boolean BL_ABS_REVERSED = true;
        public static final boolean FR_ABS_REVERSED = true;
        public static final boolean BR_ABS_REVERSED = true;

        // The physical max if motors go full speed
        public static final double MAX_METERS_PER_SEC = 8; // m/s
        public static final double MAX_RADIANS_PER_SEC = 9; // rad/s

        public static final double MAX_LINEAR_ACCEL = 7.5; // m/s/s
        public static final double MAX_ANGULAR_ACCEL = 12; // rad/s/s

    }

    public static final class AutoConstants {
        // For now keep auto speeds 1/3 of teleop
        public static final double AUTO_MAX_METERS_PER_SEC = DriveConstants.MAX_METERS_PER_SEC / 2;
        public static final double AUTO_MAX_RADIANS_PER_SEC = DriveConstants.MAX_RADIANS_PER_SEC / 2;
        public static final double AUTO_MAX_MPSS = 2;
        public static final double AUTO_MAX_ANGULAR_ACCEL = 5;
        public static final double P_AUTO_X = 4;
        public static final double P_AUTO_Y = 4;
        public static final double I_AUTO_X = 0.05;
        public static final double I_AUTO_Y = 0.05;
        public static final double P_AUTO_THETA = 3.675764653;

        public static final TrapezoidProfile.Constraints THETA_CONTROL_PROFILE = new TrapezoidProfile.Constraints(
                AUTO_MAX_RADIANS_PER_SEC,
                AUTO_MAX_ANGULAR_ACCEL);
    }

    public static final class ArmConstants {
        public static final double WRIST_CONVERSION_FACTOR = 360; // rotations to degrees
        public static final double SHOULDER_DEADZONE = 0.1;
        public static final double ELBOW_DEADZONE = 0.1;
        public static final double WRIST_DEADZONE = 0.05;

        //echo 6, response 7
        //echo 3, response 4
        //echo 0, response 1

        public static final int BOTTOM_CONE_INTAKE_SENSOR_PORT_PING = 1;
        public static final int BOTTOM_CONE_INTAKE_SENSOR_PORT_RESPONSE = 0;

        public static final int TOP_CONE_INTAKE_SENSOR_PORT_PING = 6;
        public static final int TOP_CONE_INTAKE_SENSOR_PORT_RESPONSE = 5;

        public static final int CUBE_INTAKE_SENSOR_PORT_PING = 6;
        public static final int CUBE_INTAKE_SENSOR_PORT_RESPONSE = 5;


        public static final int SHOULDER_MOTOR_LEFT = 11;
        public static final int SHOULDER_MOTOR_RIGHT = 15;
        public static final int ELBOW_MOTOR = 10;

        public static final double SHOULDER_P = 0.1;
        public static final double SHOULDER_I = 0.0;
        public static final double SHOULDER_D = 0;
        public static final double SHOULDER_F = 0.03;

        public static final double ELBOW_P = 0.36;
        public static final double ELBOW_I = 0.0001;
        public static final double ELBOW_D = 0.0;
        public static final double ELBOW_F = 0.0;

        public static final double WRIST_P = 0.55;
        public static final double WRIST_I = 0;
        public static final double WRIST_D = 0;
    
        public static final double SHOULDER_START_ANGLE = Math.toRadians(22);

        // All in meters:
        public static final double SHOULDER_TO_ELBOW = 1;
        public static final double ELBOW_TO_WRIST = 1;
        public static final double ELBOW_TO_STRING = 1;

        public static final double SHOULDER_ENCODER_TO_RAD = 0.053108;
        public static final double ELBOW_ENCODER_TO_METERS = 1;
    }

    public static final class LightConstants {
        public static final int LED_PORT = 9;
        public static final int COUNT = 30;

    }

    public static final class FieldConstants {
        public static final double FIELD_WIDTH = 8.2296;
        public static final double FIELD_LENGTH = 16.4592;

    }
}