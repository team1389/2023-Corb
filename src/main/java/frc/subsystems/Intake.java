package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase{
    private CANSparkMax bottomRoller; 
    private CANSparkMax topRoller;
    private final double intakeSpeed = 0.5;
    private final double outtakeSpeed = 1;
    
    public Intake() {
        bottomRoller = new CANSparkMax(DriveConstants.BOTTOM_INTAKE_MOTOR, MotorType.kBrushless);
        topRoller = new CANSparkMax(DriveConstants.TOP_INTAKE_MOTOR, MotorType.kBrushless);
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

    public void stop(){
        bottomRoller.set(0);
        topRoller.set(0);
    }
}
