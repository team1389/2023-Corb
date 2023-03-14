package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DriveConstants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private final double intakeSpeed = 0.5;
    private final double outtakeSpeed = 0.5;
    private CANSparkMax rollerMotor;

    public Intake() {
        //for new intake
        rollerMotor = new CANSparkMax(DriveConstants.ROLLER_MOTOR_PORT, MotorType.kBrushless);
    }

    public void runIntakeCone() {
        rollerMotor.set(intakeSpeed);
    }

    public void runIntakeCube() {
        rollerMotor.set(-intakeSpeed);
    }

    public void runOuttakeCube() {
        rollerMotor.set(outtakeSpeed);
    }

    public void runOuttakeCone() {
        rollerMotor.set(-outtakeSpeed);
    }

    public void stop() {
        rollerMotor.set(0);
    }
}
