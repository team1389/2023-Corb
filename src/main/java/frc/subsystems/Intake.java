package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase{
    private CANSparkMax bottomRoller; 
    private CANSparkMax topRoller;
    private Ultrasonic sensor, sensor2, sensor3;
    private final double intakeSpeed = 0.5;
    private final double outtakeSpeed = 1;
    
    public Intake() {
        bottomRoller = new CANSparkMax(DriveConstants.BOTTOM_INTAKE_MOTOR, MotorType.kBrushless);
        topRoller = new CANSparkMax(DriveConstants.TOP_INTAKE_MOTOR, MotorType.kBrushless);
        sensor = new Ultrasonic(ArmConstants.INTAKE_SENSOR_PORT_PING, ArmConstants.INTAKE_SENSOR_PORT_RESPONSE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake distance", getSensor());
    }

    public void runIntakeCone(){
        bottomRoller.set(-intakeSpeed);
        topRoller.set(-intakeSpeed);
    }

    public void runIntakeCube(){
        bottomRoller.set(intakeSpeed);
        topRoller.set(intakeSpeed);
    }

    public void runOuttakeCube(){
        bottomRoller.set(-outtakeSpeed);
        topRoller.set(-outtakeSpeed);
    }

    public void runOuttakeCone(){
        bottomRoller.set(outtakeSpeed);
        topRoller.set(outtakeSpeed);
    }

    public double getSensor() {
        return sensor.getRangeInches();
    }

    public void stop(){
        bottomRoller.set(0);
        topRoller.set(0);
    }
}
