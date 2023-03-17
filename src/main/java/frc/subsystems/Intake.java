package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Intake extends SubsystemBase {
    private final double intakeSpeed = 0.5;
    private final double outtakeSpeed = 0.5;
    private CANSparkMax rollerMotor;

    public Intake() {
        //for new intake
        rollerMotor = new CANSparkMax(DriveConstants.ROLLER_MOTOR_PORT, MotorType.kBrushless);
        rollerMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.setSmartCurrentLimit(40);
        rollerMotor.burnFlash();

    }

    public void runIntakeCone() {
        rollerMotor.set(-intakeSpeed);
    }

    public void runIntakeCube() {
        rollerMotor.set(intakeSpeed);
    }

    public void runOuttakeCube() {
        rollerMotor.set(-outtakeSpeed);
    }

    public void shootCube() {
        rollerMotor.set(-1);
    }

    public void runOuttakeCone() {
        rollerMotor.set(outtakeSpeed);
    }

    public void stop() {
        rollerMotor.set(0);
    }

} 
