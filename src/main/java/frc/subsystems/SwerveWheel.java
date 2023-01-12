package frc.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveWheel extends SubsystemBase {
    private CANSparkMax rotationMotor, driveMotor;
    public SwerveWheel(CANSparkMax rotationMotor, CANSparkMax driveMotor){
        driveMotor = new CANSparkMax(deviceId, type);
    }
    
}