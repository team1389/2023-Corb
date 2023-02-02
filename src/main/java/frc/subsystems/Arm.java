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
    private ArrayList<Double> shoulderSet = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));//low, mid, high
    private ArrayList<Double> elbowSet = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));//low, mid, high

    public enum Position{
        Low,
        Mid,
        High
    }

     Map<Position, Double[]> hmmmmm = new HashMap<Position, Double[] >();

    
    public Arm(){
        shoulder = new CANSparkMax(DriveConstants.SHOULDER_MOTOR, MotorType.kBrushless);
        elbow = new CANSparkMax(DriveConstants.ELBOW_MOTOR, MotorType.kBrushless);
        pidShoulder = new PIDController(sP, sI, sD);
        pidElbow = new PIDController(eP, eI, eD);

        hmmmmm.put(Position.Low, new Double[]{0.0, 0.0});
        hmmmmm.put(Position.Mid, new Double[]{0.0, 0.0});
        hmmmmm.put(Position.High, new Double[]{0.0, 0.0});
    }

    public void setArm(Position pos){
        shoulder.set(pidShoulder.calculate(getShoulderDistance(), hmmmmm.get(pos)[0]));
        elbow.set(pidElbow.calculate(getElbowDistance(), hmmmmm.get(pos)[1]));
    }

    public double getShoulderDistance(){
        return 0;
    }
    public double getElbowDistance(){
        return 0;
    }


}
