package frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DriveConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax shoulder, elbow;
    private double sP, sI, sD, eP, eI, eD;
    private PIDController pidShoulder;
    private PIDController pidElbow;
   
    public enum Position{
        Low,
        Mid,
        High
    }

    public Map<Position, Double[]> hmmmmm = new HashMap<Position, Double[] >();
    
    public Position targetPos = Position.Low;

    
    public Arm(){
        // shoulder = new CANSparkMax(DriveConstants.SHOULDER_MOTOR, MotorType.kBrushless);
        // elbow = new CANSparkMax(DriveConstants.ELBOW_MOTOR, MotorType.kBrushless);
        pidShoulder = new PIDController(sP, sI, sD);
        pidElbow = new PIDController(eP, eI, eD);

        hmmmmm.put(Position.Low, new Double[]{0.0, 0.0});
        hmmmmm.put(Position.Mid, new Double[]{0.0, 0.0});
        hmmmmm.put(Position.High, new Double[]{0.0, 0.0});
    }

    public void setArm(Position pos){
        targetPos = pos;
    }

    @Override
    public void periodic() {
        shoulder.set(pidShoulder.calculate(getShoulderDistance(), hmmmmm.get(targetPos)[0]));
        elbow.set(pidElbow.calculate(getElbowDistance(), hmmmmm.get(targetPos)[1]));
    }

    public double getShoulderDistance(){
        return 0;
    }
    public double getElbowDistance(){
        return 0;
    }


}
