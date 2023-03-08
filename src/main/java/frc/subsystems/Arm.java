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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
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

    public boolean controllerInterrupt = true;

    private double lastMovement;

    private RelativeEncoder wristEncoder, shoulderLeftEncoder, shoulderRightEncoder, elbowEncoder;
    private DutyCycleEncoder wristAbsoluteEncoder = new DutyCycleEncoder(8);

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

    public ArmPosition targetPos;
    public double shoulderTarget;
    public double elbowTarget;
    public double wristTarget;
    private double shoulderSpeed;
    private double elbowSpeed;

    // Number of steps to take to get there. Higher is smoother but slower
    private double numSteps = 20;
    private double currentStep = 0;

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
        // by shoulder and elbow are by definition, wrist is from the absolute encoder.
        positionMap.put(ArmPosition.StartingConfig, new Double[] { 0.0, 0.0, 0.38 });

        positionMap.put(ArmPosition.IntakeCube, new Double[] { 0.1, -5.25, 0.15 }); // TODO
        positionMap.put(ArmPosition.IntakeConeBottom, new Double[] { 0.0, -15.0, 0.27 }); // TODO
        positionMap.put(ArmPosition.IntakeConeTop, new Double[] { 0.0, -5.183, 0.219102976727574 }); // TODO

        positionMap.put(ArmPosition.Low, new Double[] { 0.0, 0.0, -27.0 }); // TODO
        positionMap.put(ArmPosition.MidConeBottom, new Double[] { 1.62, 11.96, -3.86 });
        positionMap.put(ArmPosition.HighConeBottom, new Double[] { 3.4, -35.78, -57.07 });
        positionMap.put(ArmPosition.MidConeTop, new Double[] { 1.56, 9.14, 0.5 });
        positionMap.put(ArmPosition.HighConeTop, new Double[] { 3.12, -22.62, -5.0 });
        positionMap.put(ArmPosition.MidCube, new Double[] { 1.347, -2.8, 0.149 });
        positionMap.put(ArmPosition.HighCube,
                new Double[] { 2.8, -6.0, 0.15 });

        setArm(ArmPosition.StartingConfig);
        shoulderTarget = positionMap.get(targetPos)[0];
        elbowTarget = positionMap.get(targetPos)[1];
        wristTarget = positionMap.get(targetPos)[2];
    }

    public double setShoulder(double pos) {
        var temp = shoulderTarget;
        shoulderTarget = pos;
        SmartDashboard.putNumber("Shoulder target", shoulderTarget);
        return temp;
    }

    public double setElbow(double pos) {
        var temp = elbowTarget;
        elbowTarget = pos;
        SmartDashboard.putNumber("Elbow target", elbowTarget);
        return temp;
    };

    public double setWrist(double pos) {
        var temp = wristTarget;
        wristTarget = pos;
        SmartDashboard.putNumber("Wrist target", wristTarget);
        return temp;
    };

    public void setArm(double shoulder, double elbow, double wrist) {
        setShoulder(shoulder);
        setElbow(elbow);
        setWrist(wrist);
    };

    public void setArm(ArmPosition pos) {
        if (this.targetPos == ArmPosition.StartingConfig) {
            lastMovement = Timer.getFPGATimestamp();
        }

        targetPos = pos;

        shoulderSpeed = (positionMap.get(targetPos)[0] - getShoulderPos()) / numSteps;
        elbowSpeed = (positionMap.get(targetPos)[1] - getElbowPos()) / numSteps;

        currentStep = 0;

        // setShoulder(positionMap.get(targetPos)[0]);
        // setElbow(positionMap.get(targetPos)[1]);
        // setWrist(positionMap.get(targetPos)[2]);
    }

    @Override
    public void periodic() {
        double shoulderPower = 0, wristPower = 0, elbowPower = 0;

        // Update targets by speed and increment step number
        if(currentStep < numSteps) {
            shoulderTarget += shoulderSpeed;
            elbowTarget += elbowSpeed;
            currentStep ++;
        }

        if (!controllerInterrupt) {
            // wait time to let elbow drop
            if (lastMovement + 0.5 < Timer.getFPGATimestamp()) {
                shoulderPower = pidShoulder.calculate(getShoulderPos(), shoulderTarget);
                moveShoulder(shoulderPower);
                
            }
            if (lastMovement + 0.2 < Timer.getFPGATimestamp()) {
                wristPower = pidWrist.calculate(getWristPos(), wristTarget);
                moveWrist(wristPower);
            }
            elbowPower = pidElbow.calculate(getElbowPos(), elbowTarget);
            moveElbow(elbowPower);
        }

        // SmartDashboard.putNumber("Shoulder power", shoulderPower);
        // SmartDashboard.putNumber("Elbow power", elbowPower);
        // SmartDashboard.putNumber("Wrist power", wristPower);

        // SmartDashboard.putNumber("Tree", Timer.getFPGATimestamp());
        // SmartDashboard.putNumber("last", lastMovement);

        SmartDashboard.putNumber("Shoulder position", getShoulderPos());
        SmartDashboard.putNumber("Elbow position", getElbowPos());
        SmartDashboard.putNumber("Wrist position", getWristPos());

        SmartDashboard.putNumber("Shoulder speed", shoulderSpeed);
        SmartDashboard.putNumber("Elbow speed", elbowSpeed);

        SmartDashboard.putNumber("Elbow target", elbowTarget);
        SmartDashboard.putNumber("Elbow target", shoulderTarget);

        SmartDashboard.putNumber("Current step", currentStep);

        // shoulderTarget = SmartDashboard.getNumber("Shoulder target", getShoulderPos());
        // elbowTarget = SmartDashboard.getNumber("Elbow target", getElbowPos());
        // wristTarget = SmartDashboard.getNumber("Wrist target", getWristPos());

    } 

    public double getWristPos() {
        return wristAbsoluteEncoder.getDistance();
    }

    // Returns encoder count
    public double getShoulderPos() {
        return shoulderLeftEncoder.getPosition();
    }

    // Returns encoder count
    public double getElbowPos() {
        return elbowEncoder.getPosition();
    }

    // Returns shoulder angle (rad)
    public double getShoulderAngle() {
        return shoulderLeftEncoder.getPosition() * ArmConstants.SHOULDER_ENCODER_TO_RAD;
    }

    // Returns elbow angle (rad)
    public double getElbowAngle() {

        // String length (meters)
        double length = elbowEncoder.getPosition() * ArmConstants.ELBOW_ENCODER_TO_METERS;

        // Law of cosines
        double cosAngle = (length * length) - (ArmConstants.SHOULDER_TO_ELBOW * ArmConstants.SHOULDER_TO_ELBOW)
                - (ArmConstants.ELBOW_TO_STRING * ArmConstants.ELBOW_TO_STRING);
        cosAngle = cosAngle / ((-2) * ArmConstants.SHOULDER_TO_ELBOW * ArmConstants.ELBOW_TO_STRING);

        return Math.acos(cosAngle);
    }

    // Get length given angle (radians)
    public double getElbowLength(double angle) {
        // Law of cosines again
        double temp = (ArmConstants.SHOULDER_TO_ELBOW * ArmConstants.SHOULDER_TO_ELBOW)
                + (ArmConstants.ELBOW_TO_STRING * ArmConstants.ELBOW_TO_STRING);
        temp = temp - (2 * ArmConstants.SHOULDER_TO_ELBOW * ArmConstants.ELBOW_TO_STRING * Math.cos(angle));
        return Math.sqrt(temp);
    }

    // Returns (x, y) position of elbow joint relative to shoulder
    public double[] getElbowPosition() {
        double x = Math.sin(getShoulderAngle()) * ArmConstants.SHOULDER_TO_ELBOW;
        double y = -Math.cos(getShoulderAngle()) * ArmConstants.SHOULDER_TO_ELBOW;

        return new double[] { x, y };
    }

    // Returns (x, y) position of wrist joint relative to shoulder
    public double[] getTipPosition() {
        double[] elbowPos = getElbowPosition();
        double x = -Math.cos(getElbowAngle()) * ArmConstants.ELBOW_TO_WRIST;
        double y = Math.sin(getElbowAngle()) * ArmConstants.ELBOW_TO_WRIST;
        double shoulderAngle = getShoulderAngle();

        // Do some trig I think this works
        double xRotated = x * Math.cos(shoulderAngle) - y * Math.sin(shoulderAngle);
        double yRotated = x * Math.sin(shoulderAngle) + y * Math.cos(shoulderAngle);

        return new double[] { elbowPos[0] + xRotated, elbowPos[1] + yRotated };
    }

    // Returns target (x, y) for wrist based on target angles
    public double[] getPosFromAngles(double shoulderAngle, double elbowAngle) {
        // It's mathin' time
        double elbowX = Math.sin(shoulderAngle) * ArmConstants.SHOULDER_TO_ELBOW;
        double elbowY = -Math.cos(shoulderAngle) * ArmConstants.SHOULDER_TO_ELBOW;
        double x = -Math.cos(elbowAngle) * ArmConstants.ELBOW_TO_WRIST;
        double y = Math.sin(elbowAngle) * ArmConstants.ELBOW_TO_WRIST;
        double xRotated = x * Math.cos(shoulderAngle) - y * Math.sin(shoulderAngle);
        double yRotated = x * Math.sin(shoulderAngle) + y * Math.cos(shoulderAngle);

        return new double[] { elbowX + xRotated, elbowY + yRotated };
    }

    // Returns gradient component for a joint given vector from joint to tip and
    // vector from tip to target
    public double getGradient(double[] jointToTip, double[] tipToTarget) {
        // First get vector by which turning joint will move tip
        // Do this by crossing joint to tip with axis of rotation, (0,0,1) for both
        // joints in this case
        double[] movementVector = cross(jointToTip, new double[] { 0, 0, 1 });

        // Then dot this movement with the desired direction of motion
        return dot(movementVector, tipToTarget);
    }

    // Vector methods
    // Only for 2d vectors
    public double dot(double[] v1, double[] v2) {
        return v1[0] * v2[0] + v1[1] * v2[1];
    }

    // Only for 3d vectors
    public double[] cross(double[] v1, double[] v2) {
        double[] toReturn = new double[3];
        toReturn[0] = v1[1] * v2[2] - v1[2] * v2[1];
        toReturn[1] = v1[0] * v2[2] - v1[2] * v2[0];
        toReturn[2] = v1[0] * v2[1] - v1[1] * v2[0];

        return toReturn;
    }

    // Debugging methods below:
    public void moveShoulder(double power) {
        power = MathUtil.clamp(power, -0.1, 0.6);
        shoulderLeft.set(power);
        shoulderRight.set(power);
    }

    public void moveElbow(double power) {
        if (positionMap.get(targetPos).length > 3) {
            power = (getElbowPos() < positionMap.get(targetPos)[3] ? 0 : power);
        }
        elbow.set(MathUtil.clamp(power, -0.45, 0.45));
    }

    public void moveWrist(double power) {
        // power = MathUtil.clamp(power, -1, 1);
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
