package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.util.SizeLimitedQueue;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private final double intakeSpeed = 0.5;
    private final double outtakeSpeed = 0.5;
    private final double cubeResistance = 2;
    private final double coneResistance = 2;
    
    private CANSparkMax bottomRoller;
    private CANSparkMax topRoller;
    private Ultrasonic cubeSensor, coneSensorBottom, coneSensorTop;
    private double sensorThreshold = 10;


    private int temp = 0;
    SizeLimitedQueue topSizeLimitedQueue = new SizeLimitedQueue(5);
    SizeLimitedQueue bottomSizeLimitedQueue = new SizeLimitedQueue(5);
    

    public Intake() {
        bottomRoller = new CANSparkMax(DriveConstants.BOTTOM_INTAKE_MOTOR, MotorType.kBrushless);
        topRoller = new CANSparkMax(DriveConstants.TOP_INTAKE_MOTOR, MotorType.kBrushless);

        // coneSensorBottom = new
        // Ultrasonic(ArmConstants.BOTTOM_CONE_INTAKE_SENSOR_PORT_PING,
        // ArmConstants.BOTTOM_CONE_INTAKE_SENSOR_PORT_RESPONSE);
        // coneSensorTop = new Ultrasonic(ArmConstants.TOP_CONE_INTAKE_SENSOR_PORT_PING,
        // ArmConstants.TOP_CONE_INTAKE_SENSOR_PORT_RESPONSE);
        cubeSensor = new Ultrasonic(ArmConstants.CUBE_INTAKE_SENSOR_PORT_PING,
                ArmConstants.CUBE_INTAKE_SENSOR_PORT_RESPONSE);

        // coneSensorBottom.setEnabled(true);
        // coneSensorTop.setEnabled(true);
        cubeSensor.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Bottom Cone Intake", getBottomCone());
        // SmartDashboard.putBoolean("Top Cone Intake", getTopCone());
        SmartDashboard.putBoolean("Cube Intake", hasCube());

        topSizeLimitedQueue.add(Math.abs(topRoller.getEncoder().getVelocity()));
        bottomSizeLimitedQueue.add(Math.abs(bottomRoller.getEncoder().getVelocity()));
        
        if (hasBottomCone() == true || hasTopCone() == true || hasCube() == true) {
            temp++;
            // if (temp > 5) {
                // stop();
            // }
        } else {
            temp = 0;
        }


    }

    public void runIntakeCone() {
        bottomRoller.set(-intakeSpeed);
        topRoller.set(-intakeSpeed);
    }

    public void runIntakeCube() {
        bottomRoller.set(intakeSpeed);
        topRoller.set(intakeSpeed);
    }

    public void runOuttakeCube() {
        bottomRoller.set(-outtakeSpeed);
        topRoller.set(-outtakeSpeed);
    }

    public void runOuttakeCone() {
        bottomRoller.set(outtakeSpeed);
        topRoller.set(outtakeSpeed);
    }

    public boolean hasCube() {
        return topSizeLimitedQueue.getAverageDerivative() + bottomSizeLimitedQueue.getAverageDerivative() < -cubeResistance;
    }

    public boolean hasBottomCone() {
        return topSizeLimitedQueue.getAverageDerivative() + bottomSizeLimitedQueue.getAverageDerivative() < -coneResistance;
    }

    public boolean hasTopCone() {
        return topSizeLimitedQueue.getAverageDerivative() + bottomSizeLimitedQueue.getAverageDerivative() < -coneResistance;
    }

    public void stop() {
        bottomRoller.set(0);
        topRoller.set(0);
    }
}
