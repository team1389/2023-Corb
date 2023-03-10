
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ModuleConstants;

public class SwerveModule extends SubsystemBase{

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
                            boolean absoluteEncoderReversed, double anglularOffset) {

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
        // turnPidController.setI(ModuleConstants.I_TURNING);
        turnPidController.setD(ModuleConstants.D_TURNING);

        //Uncomment to tune pid from SmartDashboard
        // SmartDashboard.putNumber("Turning P", ModuleConstants.P_TURNING);
        // SmartDashboard.putNumber("Turning I", ModuleConstants.I_TURNING);
        // SmartDashboard.putNumber("Turning D", ModuleConstants.D_TURNING);


        drivePidController.setP(ModuleConstants.P_DRIVE);
        drivePidController.setFF(1/ModuleConstants.DRIVE_FREE_MAX_SPEED_MPS);


        turnPidController.setOutputRange(-1, 1);
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

    public void setDesiredState(SwerveModuleState state, boolean deadzone) {
        // If the speed is 0 (basically if the driver isn't touching joystick) don't snap motors to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.05 && deadzone) {
            stop();
            return;
        }

        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedDesiredState.angle = state.angle.plus(Rotation2d.fromRadians(angularOffset));
        
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(turnEncoder.getPosition()));
        
        drivePidController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turnPidController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    @Override
    public void periodic() {
        //Uncomment to tune pid from SmartDashboard
        // turnPidController.setP(SmartDashboard.getNumber("Turning P", 0.01));
        // turnPidController.setI(SmartDashboard.getNumber("Turning I", 0.00001));
        // turnPidController.setD(SmartDashboard.getNumber("Turning D", 0.0005));

    }
}