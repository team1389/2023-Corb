package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;
import frc.util.SizeLimitedQueue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    private CANSparkMax rollerMotor;


    SizeLimitedQueue topSizeLimitedQueue = new SizeLimitedQueue(5);
    SizeLimitedQueue bottomSizeLimitedQueue = new SizeLimitedQueue(5);
    

    public Intake() {
        bottomRoller = new CANSparkMax(DriveConstants.BOTTOM_INTAKE_MOTOR, MotorType.kBrushless);
        topRoller = new CANSparkMax(DriveConstants.TOP_INTAKE_MOTOR, MotorType.kBrushless);

        //for new intake
        rollerMotor = new CANSparkMax(DriveConstants.PLACEHOLDER_PORT, MotorType.kBrushless);
    }

    //intake
    public void corbEat() {
        rollerMotor.set(intakeSpeed);
    }

    //outtake
    public void corbFire() {
        rollerMotor.set(outtakeSpeed);
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
