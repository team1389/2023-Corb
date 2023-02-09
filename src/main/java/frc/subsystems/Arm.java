package frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax shoulder, elbow, wrist;
    private double sP, sI, sD, eP, eI, eD, wP, wI, wD;
    private PIDController pidShoulder;
    private PIDController pidElbow;
    private PIDController pidWrist;
    
    private RelativeEncoder wristEncoder;

    public enum Position{
        Low,
        Mid,
        High
    }

    public Map<Position, Double[]> hmmmmm = new HashMap<Position, Double[] >();
    
    public Position targetPos = Position.Low;

    
    public Arm(){
        shoulder = new CANSparkMax(DriveConstants.SHOULDER_MOTOR, MotorType.kBrushless);
        elbow = new CANSparkMax(DriveConstants.ELBOW_MOTOR, MotorType.kBrushless);
        wrist = new CANSparkMax(DriveConstants.WRIST_MOTOR, MotorType.kBrushless);
        pidShoulder = new PIDController(sP, sI, sD);
        pidElbow = new PIDController(eP, eI, eD);
        pidWrist = new PIDController(wP, wI, wD);
        wristEncoder = wrist.getEncoder();

        wristEncoder.setPositionConversionFactor(ArmConstants.WRIST_CONVERSION_FACTOR);

        // Shoulder, elbow, wrist
        hmmmmm.put(Position.Low, new Double[]{0.0, 0.0, 0.0});
        hmmmmm.put(Position.Mid, new Double[]{0.0, 0.0, 0.0});
        hmmmmm.put(Position.High, new Double[]{0.0, 0.0, 0.0});
    }

    public void setArm(Position pos){
        targetPos = pos;
    }

    @Override
    public void periodic() {
        shoulder.set(pidShoulder.calculate(getShoulderDistance(), hmmmmm.get(targetPos)[0]));
        elbow.set(pidElbow.calculate(getElbowDistance(), hmmmmm.get(targetPos)[1]));
        wrist.set(pidWrist.calculate(wristEncoder.getPosition(), hmmmmm.get(targetPos)[2]));

        SmartDashboard.putNumber("Shoulder position", getShoulderDistance());
        SmartDashboard.putNumber("Elbow position", getElbowDistance());
        SmartDashboard.putNumber("Wrist position", wristEncoder.getPosition());
    }

    public double getShoulderDistance(){
        return 0;
    }
    public double getElbowDistance(){
        return 0;
    }

    // Debugging methods below:
    public void moveShoulder(double power) {
        shoulder.set(power);
    }

    public void moveElbow(double power) {
        elbow.set(power);
    }
}
