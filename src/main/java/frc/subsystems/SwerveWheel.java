package frc.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveWheel extends SubsystemBase {

  //Creating motor and PID controller variables
  private CANSparkMax rotationMotor, driveMotor;
  private SparkMaxPIDController rotationPIDController, drivePIDController;

  //Doesn't reset between matches, unlike the built in relative encoders. Abs Encoder Pog
  public CANCoder rotateAbsEncoder;

  //Multiplied by the native output units (-1 to 1) to find position
  private final double ROTATION_POSITION_CONVERSION_FACTOR = 1 / (5.33 * 7);

  //Factor between RPM and m/s
  private final double DRIVE_VELOCITY_CONVERSION_FACTOR =
    (3 * Math.PI * 0.0254) / (60 * 5.25);

  //Create PID coefficients
  public double rotateP = 0.0055; //0.025 //Zach Note: Ask Max why that value is commented out
  public double rotateI = 0.0000;
  public double rotateD = 0;

  public double driveP = 0.0007667;
  public double driveI = 0.0000005;
  public double driveD = 0;

  private double currentTarget;

  public SwerveWheel(
    int driveMotorPort,
    int rotateMotorPort,
    int rotateEncoderPort
  ) {
    //Instantiate motors/sensors
    driveMotor =
      new CANSparkMax(driveMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    rotateMotor =
      new CANSparkMax(
        rotateMotorPort,
        CANSparkMaxLowLevel.MotorType.kBrushless
      );
    rotateAbsEncoder = new CANCoder(rotateEncoderPort);

    rotateAbsEncoder.setPositionToAbsolute();

    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rotateMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    driveMotor.setInverted(false);

    //Assign PID controllers' parameters
    rotatePIDController = rotateMotor.getPIDController();

    //Set the kP, kI, and kD values for the rotatePIDController
    rotatePIDController.setP(rotateP);
    rotatePIDController.setI(rotateI);
    rotatePIDController.setD(rotateD);

    rotatePIDController.setSmartMotionAccelStrategy(
      SparkMaxPIDController.AccelStrategy.kTrapezoidal,
      0
    );

    rotateMotor
      .getEncoder()
      .setPositionConversionFactor(ROTATION_POSITION_CONVERSION_FACTOR * 360);
    rotateMotor.getEncoder().setPosition(0);

    drivePIDController = driveMotor.getPIDController();

    //Set the kP, kI, and kD values for the rotatePIDController
    drivePIDController.setP(driveP);
    drivePIDController.setI(driveI);
    drivePIDController.setD(driveD);
  }

  //Sets the drive motor power
  public void setPower(double power) {
    driveMotor.set(power);
  }

  //sets the speed in meters per second
  //Zach Note: Doesn't seem to be sending any errors what so ever rn so check later
  public void setSpeed(double speedMeterPerSecond) {
    drivePIDController.setReference(
      speedMetersPerSecond / DRIVE_VELOCITY_CONVERSION_FACTOR,
      CANSparkMax.ControlType.kVelocity
    );
  }

  //Setting the Angle for the Rotation Motor
  public double setAngle(double angle) {
    double currentAngle = rotateMotor.getEncoder().getPosition();
    double setpointAngle = closestAngle(currentAngle, angle);
    double setpointAngleFlipped = closestAngle(currentAngle, angle + 180.0);

    //If the closest angle to setpoint is shorter
    if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
      driveMotor.setInverted(false);
      rotatePIDController.setReference(
        currentAngle + setpointAngle,
        CANSparkMax.ControlType.kPosition,
        0
      );
    }
    //If the closest angle to setpoint + 180 is shorter
    else {
      driveMotor.setInverted(true);
      rotatePIDController.setReference(
        currentAngle + setpointAngleFlipped,
        CANSparkMax.ControlType.kPosition,
        0
      );
    }

    currentTarget = currentAngle + setpointAngle;
    return currentTarget;
  }

  //Set the state of the wheel with a SwerveModuleState
  public void setState(SwerveModuleState state) {
    //Reverse the angle to match default from rotation encoders
    setAngle(-state.angle.getDegrees());

    setSpeed(state.speedMetersPerSecond);
  }

  //Set the relative encoder to its wheel's actual angle
  public void coordinateRelativeEncoder() {
    double absAngle = rotateAbsEncoder.getAbsolutePosition();
    rotateMotor.getEncoder().setPosition(-absAngle);
  }

  //Get the closest angle between the given angles.
  private static double closestAngle(double a, double b) {
    //Get direction
    double dir = (b % 360.0) - (a % 360.0);

    //Convert from -360 to 360 to -180 to 180
    if (Math.abs(dir) > 180.0) {
      dir = -(Math.signum(dir) * 360.0) + dir;
    }
    return dir;
  }

  //Only run this when training the angle. DO NOT RUN IN MATCHES
  public void resetAbsEncoder() {
    rotateAbsEncoder.setPosition(0);
    rotateMotor.getEncoder().setPosition(0);
  }

  public SwerveModuleState getState() {
    //Return the current module position and speed
    if (driveMotor.getInverted()) {
      return new SwerveModuleState(
        -driveMotor.getEncoder().getVelocity() *
        DRIVE_VELOCITY_CONVERSION_FACTOR,
        Rotation2d.fromDegrees(-rotateMotor.getEncoder().getPosition())
      );
    }
    return new SwerveModuleState(
      driveMotor.getEncoder().getVelocity() * DRIVE_VELOCITY_CONVERSION_FACTOR,
      Rotation2d.fromDegrees(-rotateMotor.getEncoder().getPosition())
    );
  }

  //returns pid value
  public double[] getPID() {
    double[] pidConstants = { rotateP, rotateI, rotateD };
    return pidConstants;
  }

  //returns TargetAngle Value
  public double getTargetAngle() {
    return currentTarget;
  }
}
