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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax shoulderLeft, shoulderRight, elbow, wrist;
    private double sD, wI, wD;

    private double sP = 1;
    private double sI = 0.02;

    private double eP = 0.6;

    private double wP = 0.09;
    private PIDController pidShoulder;
    private PIDController pidElbow;
    private PIDController pidWrist;
    
    private RelativeEncoder wristEncoder, shoulderLeftEncoder, shoulderRightEncoder, elbowEncoder;

    public enum ArmPosition{
        Intake,
        Low,
        MidCone,
        HighCone,
        MidCube,
        HighCube,
        StartingConfig
    }

    public Map<ArmPosition, Double[]> positionMap = new HashMap<ArmPosition, Double[] >();
    
    public ArmPosition targetPos = ArmPosition.Low;

    
    public Arm(){
        shoulderLeft = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);

        elbow = new CANSparkMax(ArmConstants.ELBOW_MOTOR, MotorType.kBrushless);
        wrist = new CANSparkMax(DriveConstants.WRIST_MOTOR, MotorType.kBrushless);

        elbow.setIdleMode(IdleMode.kBrake);
        shoulderLeft.setIdleMode(IdleMode.kBrake);
        shoulderRight.setIdleMode(IdleMode.kBrake);
        wrist.setIdleMode(IdleMode.kBrake);

        shoulderLeft.setInverted(false);
        shoulderRight.setInverted(false);

        pidShoulder = new PIDController(sP, sI, sD);
        pidElbow = new PIDController(eP, 0, 0);
        pidWrist = new PIDController(wP, wI, wD);

        wristEncoder = wrist.getEncoder();
        shoulderLeftEncoder = shoulderLeft.getEncoder();
        shoulderRightEncoder = shoulderLeft.getEncoder();
        elbowEncoder = elbow.getEncoder();

        wristEncoder.setPosition(0);
        shoulderLeftEncoder.setPosition(0);
        elbowEncoder.setPosition(0);


        // Shoulder, elbow, wrist, min elbow
        positionMap.put(ArmPosition.Low, new Double[]{0.0, 0.0, -27.0});
        positionMap.put(ArmPosition.Intake, new Double[]{0.0, -15.0, 0.0});
        positionMap.put(ArmPosition.MidCone, new Double[]{0.0, 0.0, 0.0});
        positionMap.put(ArmPosition.HighCone, new Double[]{0.0, 0.0, 0.0, 0.0});
        positionMap.put(ArmPosition.MidCube, new Double[]{1.2, 0.0, 0.0});
        positionMap.put(ArmPosition.HighCube, new Double[]{2.9, 0.0, 0.0});
        positionMap.put(ArmPosition.StartingConfig, new Double[]{0.0, 0.0, 0.0});
    }

    public void setArm(ArmPosition pos){
        targetPos = pos;
    }

    @Override
    public void periodic() {
        double shoulderPower = pidShoulder.calculate(getShoulderDistance(), positionMap.get(targetPos)[0]);
        double elbowPower = pidElbow.calculate(getElbowDistance(), positionMap.get(targetPos)[1]);
        double wristPower = pidWrist.calculate(wristEncoder.getPosition(), positionMap.get(targetPos)[2]);
        // moveShoulder(shoulderPower);
        // moveElbow(elbowPower);
        // moveWrist(wristPower);

        SmartDashboard.putNumber("Shoulder power", shoulderPower);
        SmartDashboard.putNumber("Elbow power", elbowPower);
        SmartDashboard.putNumber("Wrist power", wristPower);

        SmartDashboard.putNumber("Shoulder target", positionMap.get(targetPos)[0]);
        SmartDashboard.putNumber("Elbow target", positionMap.get(targetPos)[1]);
        SmartDashboard.putNumber("Wrist target", positionMap.get(targetPos)[2]);

        SmartDashboard.putNumber("Shoulder position", getShoulderDistance());
        SmartDashboard.putNumber("Elbow position", getElbowDistance());
        SmartDashboard.putNumber("Wrist position", wristEncoder.getPosition());
    }

    public double getShoulderDistance(){
        return shoulderLeftEncoder.getPosition();
    }
    public double getElbowDistance(){
        return elbowEncoder.getPosition();
    }

    // Debugging methods below:
    public void moveShoulder(double power) {
        power = MathUtil.clamp(power, -1, 1);
        shoulderLeft.set(power);
        shoulderRight.set(power);
    }

    public void moveElbow(double power) {
        if(positionMap.get(targetPos).length > 3) {
            power = (getElbowDistance() < positionMap.get(targetPos)[3] ? 0 : power);
        }
        elbow.set(MathUtil.clamp(power, -0.9, 0.9));
    }

    public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.25, 0.25);
        wrist.set(power);
    }

    public boolean getAtPosition() {
        Double[] targetLengths = positionMap.get(targetPos);
        
        boolean atPosition = 
            Math.abs(getShoulderDistance()-targetLengths[0]) < ArmConstants.SHOULDER_DEADZONE &&
            Math.abs(getElbowDistance()-targetLengths[1]) < ArmConstants.ELBOW_DEADZONE &&
            Math.abs(wristEncoder.getPosition()-targetLengths[2]) < ArmConstants.WRIST_DEADZONE;

        return atPosition;
    }
}
