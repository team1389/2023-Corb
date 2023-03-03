package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase{
    private CANSparkMax bottomRoller; 
    private CANSparkMax topRoller;
    private Ultrasonic cubeSensor, coneSensorBottom, coneSensorTop;
    private double sensorThreshold = 10;
    private final double intakeSpeed = 0.5;
    private final double outtakeSpeed = 0.5;
    
    public Intake() {
        bottomRoller = new CANSparkMax(DriveConstants.BOTTOM_INTAKE_MOTOR, MotorType.kBrushless);
        topRoller = new CANSparkMax(DriveConstants.TOP_INTAKE_MOTOR, MotorType.kBrushless);
        coneSensorBottom = new Ultrasonic(ArmConstants.BOTTOM_CONE_INTAKE_SENSOR_PORT_PING, ArmConstants.BOTTOM_CONE_INTAKE_SENSOR_PORT_RESPONSE);
        coneSensorTop = new Ultrasonic(ArmConstants.TOP_CONE_INTAKE_SENSOR_PORT_PING, ArmConstants.TOP_CONE_INTAKE_SENSOR_PORT_RESPONSE);
        cubeSensor = new Ultrasonic(ArmConstants.CUBE_INTAKE_SENSOR_PORT_PING, ArmConstants.CUBE_INTAKE_SENSOR_PORT_RESPONSE);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Cone bottom distance", getBottomCone());
        SmartDashboard.putNumber("Cone Top distance", getTopCone());
        SmartDashboard.putNumber("Cube distance", getCube());

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

    public double getCube() {
        return cubeSensor.getRangeInches();
    }

    public double getBottomCone() {
        return coneSensorBottom.getRangeInches();
    }
    public double getTopCone() {
        return coneSensorTop.getRangeInches();
    }

    public void stop(){
        bottomRoller.set(0);
        topRoller.set(0);
    }
}
