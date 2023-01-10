package frc.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

public class SwerveWheel extends SubsystemBase {
    //Creating motor and PID controller variables
    private CANSparkMax rotationMotor, driveMotor;
    private SparkMaxPIDController rotationPIDController, drivePIDController;

    //Doesn't reset between matches, unlike the built in relative encoders. Abs Encoder Pog
    public CANCoder rotateAbsEncoder;

    //Multiplied by the native output units (-1 to 1) to find position
    private final double ROTATION_POSITION_CONVERSION_FACTOR = 1/(5.33 * 7);

    //Factor between RPM and m/s
    private final double DRIVE_VELOCITY_CONVERSION_FACTOR = (3 * Math.PI * 0.0254) / (60 * 5.25);

    
}
