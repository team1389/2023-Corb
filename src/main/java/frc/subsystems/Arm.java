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
import frc.robot.RobotMap;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;

public class Arm extends SubsystemBase {
    private CANSparkMax shoulderLeft, shoulderRight, elbow, wrist;

    private PIDController pidShoulder;
    private PIDController pidElbow;
    private PIDController pidWrist;
    public boolean controllerInterupt;

    private RelativeEncoder wristEncoder, shoulderLeftEncoder, shoulderRightEncoder, elbowEncoder;

    public enum ArmPosition {
        StartingConfig,
        IntakeCube,
        IntakeConeBottom,
        IntakeConeTop,
        Low,
        MidConeTop,
        HighConeTop,
        MidConeBottom,
        HighConeBottom,
        MidCube,
        HighCube
    }

    public Map<ArmPosition, Double[]> positionMap = new HashMap<ArmPosition, Double[]>();

    public ArmPosition targetPos = ArmPosition.StartingConfig;

    public Arm() {
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

        pidShoulder = new PIDController(RobotMap.ArmConstants.SHOULDER_P, RobotMap.ArmConstants.SHOULDER_I,
                RobotMap.ArmConstants.SHOULDER_D);
        pidElbow = new PIDController(RobotMap.ArmConstants.ELBOW_P, RobotMap.ArmConstants.ELBOW_I,
                RobotMap.ArmConstants.ELBOW_D);
        pidWrist = new PIDController(RobotMap.ArmConstants.WRIST_P, RobotMap.ArmConstants.WRIST_I,
                RobotMap.ArmConstants.WRIST_D);

        wristEncoder = wrist.getEncoder();
        shoulderLeftEncoder = shoulderLeft.getEncoder();
        shoulderRightEncoder = shoulderLeft.getEncoder();
        elbowEncoder = elbow.getEncoder();

        wristEncoder.setPosition(0);
        shoulderLeftEncoder.setPosition(0);
        elbowEncoder.setPosition(0);

        // Shoulder, elbow, wrist
        positionMap.put(ArmPosition.StartingConfig, new Double[] { 0.0, 0.0, 0.0 }); // by definition

        positionMap.put(ArmPosition.IntakeCube, new Double[] { -0.009, 15.51, -31.22}); // TODO
        positionMap.put(ArmPosition.IntakeConeBottom, new Double[] { 0.0, -15.0, 0.0 }); // TODO
        positionMap.put(ArmPosition.IntakeConeTop, new Double[] {0.0161, 13.74, -27.0}); // TODO



        positionMap.put(ArmPosition.Low, new Double[] { 0.0, 0.0, -27.0 }); // TODO
        positionMap.put(ArmPosition.MidConeBottom, new Double[] { 1.62, 11.96, -3.86 });
        positionMap.put(ArmPosition.HighConeBottom, new Double[] { 3.4, -35.78, -57.07 });
        positionMap.put(ArmPosition.MidConeTop, new Double[] { 1.56, 9.14, 0.5 });
        positionMap.put(ArmPosition.HighConeTop, new Double[] { 3.12, -22.62, -5.0 });
        positionMap.put(ArmPosition.MidCube, new Double[] { 0.79, 8.71, 1.36 });
        positionMap.put(ArmPosition.HighCube, new Double[] { 2.46, 17.44, 1.36 });
    }

    public void setArm(ArmPosition pos) {
        targetPos = pos;
    }

    @Override
    public void periodic() {
        double shoulderPower = pidShoulder.calculate(getShoulderPos(), positionMap.get(targetPos)[0]);
        double elbowPower = pidElbow.calculate(getElbowPos(), positionMap.get(targetPos)[1]);
        double wristPower = pidWrist.calculate(getWristPos(), positionMap.get(targetPos)[2]);

        if(!controllerInterupt) {
            moveShoulder(shoulderPower);
            moveElbow(elbowPower);
            moveWrist(wristPower);
        }

        SmartDashboard.putNumber("Shoulder power", shoulderPower);
        SmartDashboard.putNumber("Elbow power", elbowPower);
        SmartDashboard.putNumber("Wrist power", wristPower);

        SmartDashboard.putNumber("Shoulder target", positionMap.get(targetPos)[0]);
        SmartDashboard.putNumber("Elbow target", positionMap.get(targetPos)[1]);
        SmartDashboard.putNumber("Wrist target", positionMap.get(targetPos)[2]);

        SmartDashboard.putNumber("Shoulder position", getShoulderPos());
        SmartDashboard.putNumber("Elbow position", getElbowPos());
        SmartDashboard.putNumber("Wrist position", getWristPos());
    }

    private double getWristPos() {
        return wristEncoder.getPosition();
    }

    public double getShoulderPos() {
        return shoulderLeftEncoder.getPosition();
    }

    public double getElbowPos() {
        return elbowEncoder.getPosition();
    }

    // Debugging methods below:
    public void moveShoulder(double power) {
        power = MathUtil.clamp(power, -1, 1);
        shoulderLeft.set(power);
        shoulderRight.set(power);
    }

    public void moveElbow(double power) {
        if (positionMap.get(targetPos).length > 3) {
            power = (getElbowPos() < positionMap.get(targetPos)[3] ? 0 : power);
        }
        elbow.set(MathUtil.clamp(power, -0.9, 0.9));
    }

    public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.25, 0.25);
        wrist.set(power);
    }

    public boolean getAtPosition() {
        Double[] targetLengths = positionMap.get(targetPos);

        boolean atPosition = Math.abs(getShoulderPos() - targetLengths[0]) < ArmConstants.SHOULDER_DEADZONE &&
                Math.abs(getElbowPos() - targetLengths[1]) < ArmConstants.ELBOW_DEADZONE &&
                Math.abs(getWristPos() - targetLengths[2]) < ArmConstants.WRIST_DEADZONE;

        return atPosition;
    }

    public void resetEncoders() {
        shoulderLeftEncoder.setPosition(0);
        shoulderRightEncoder.setPosition(0);
        elbowEncoder.setPosition(0);
        wristEncoder.setPosition(0);
    }
}
