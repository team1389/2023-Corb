
package frc.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap.ModuleConstants;

public class SwerveModule {

    public final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;

    private final AbsoluteEncoder turnEncoder;

    private final SparkMaxPIDController drivePidController;
    private final SparkMaxPIDController turnPidController;

    private final boolean absoluteEncoderReversed;

    public double targetAngle, targetSpeed;

    public double angularOffset;

    // Instatiate new module with given ports and inversions
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed, double anglularOffset) {

        this.angularOffset = anglularOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();

        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        turnEncoder.setInverted(absoluteEncoderReversed);

        // turnEncoder.setZeroOffset(angleOffset * ModuleConstants.DRIVE_ROTATIONS_TO_METERS);

        driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ROTATIONS_TO_METERS);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_RPM_TO_METERS_PER_SEC);
        turnEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ROTATIONS_TO_RAD);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_RPM_TO_RAD_PER_SEC);

        drivePidController = driveMotor.getPIDController();
        turnPidController = turnMotor.getPIDController();

        drivePidController.setFeedbackDevice(driveEncoder);
        turnPidController.setFeedbackDevice(turnEncoder);

        turnPidController.setP(ModuleConstants.P_TURNING);
        drivePidController.setP(ModuleConstants.P_DRIVE);
        drivePidController.setFF(1/ModuleConstants.DRIVE_FREE_MAX_SPEED_MPS);


        turnPidController.setOutputRange(-0.75, 0.75);
        drivePidController.setOutputRange(-1, 1);

        turnPidController.setPositionPIDWrappingEnabled(true);
        turnPidController.setPositionPIDWrappingMinInput(0);
        turnPidController.setPositionPIDWrappingMaxInput(2*Math.PI);

        // Braking mode
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(ModuleConstants.TURN_CURRENT_LIMIT);

        // At boot reset relative encoders to absolute
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }



    // Set drive encoder to 0 and turning encoder to match absolute
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        // turningEncoder.setPosition(getAbsolutePosition() * 2 * Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition() - angularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition() - angularOffset));
    }

    public void setDesiredState(SwerveModuleState state) {
        // If the speed is 0 (basically if the driver isn't touching joystick) don't snap motors to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.05) {
            stop();
            return;
        }

        // Optimize to see if turning to opposite angle and running backwards is faster
        state = SwerveModuleState.optimize(state, getState().angle);

        // Set motors, using the pid controllers for each motor
        targetAngle = state.angle.getDegrees();

        drivePidController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turnPidController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}