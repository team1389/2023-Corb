package frc.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;

public class Arm extends SubsystemBase {
    private CANSparkMax shoulderLeft, shoulderRight, elbow, wrist;

    private PIDController  pidShoulder;
    private PIDController pidElbow;
    private PIDController pidWrist;

    public boolean controllerInterrupt = false;

    private double lastMovement;

    private RelativeEncoder shoulderLeftEncoder, shoulderRightEncoder, wristEncoder;

    public enum ArmPosition {
        StartingConfig,
        IntakeCube,
        IntakeCone,
        Low,
        MidCone,
        HighCone,
        MidCube,
        HighCube,
        IntakeConeFeeder,
        AboveMidConeTop
    }

    public Map<ArmPosition, Double[]> positionMap = new HashMap<ArmPosition, Double[]>();

    public ArmPosition targetPos = ArmPosition.StartingConfig;
    public ArmPosition lastPose;
    public double shoulderTarget;
    public double elbowTarget;
    public double wristTarget;
    private double shoulderSpeed;
    private double elbowSpeed;
    private SparkMaxAbsoluteEncoder absElbowEncoder;
    private double absWristOffset = -0.005; // From 0.38
    DigitalInput limitSwitch = new DigitalInput(0);


    // Number of steps to take to get there. Higher is smoother but slower

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


        shoulderLeftEncoder = shoulderLeft.getEncoder();
        shoulderLeftEncoder.setPositionConversionFactor(1);
        shoulderRightEncoder = shoulderLeft.getEncoder();
        wristEncoder = wrist.getEncoder();
        absElbowEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        shoulderLeftEncoder.setPosition(0);

        // pidShoulder.setFeedbackDevice(shoulderLeftEncoder);

        // absElbowEncoder.setInverted(true);

        // Shoulder, elbow, wrist
        // Shoulder and elbow are relative to start, wrist is absolute
        positionMap.put(ArmPosition.StartingConfig, new Double[] { 0.0, 2*Math.PI, 0.0 });

        positionMap.put(ArmPosition.IntakeCube, new Double[] { 0.0, 4.7808, 0.0 });
        positionMap.put(ArmPosition.IntakeCone, new Double[] { 0.0, 4.90, 0.0 });

        positionMap.put(ArmPosition.Low, new Double[] { 0.0, 0.0, 0.0 }); // TODO
        positionMap.put(ArmPosition.MidCone, new Double[] { 16.9, 4.278, 0.4725 });
        positionMap.put(ArmPosition.HighCone, new Double[] { 24.6, 3.628, 0.4275 });
        positionMap.put(ArmPosition.IntakeConeFeeder, new Double[] { 16.556, 4.211, 0.2025 });
        positionMap.put(ArmPosition.MidCube, new Double[] { 0.0, 5.87, 0.0 });
        positionMap.put(ArmPosition.HighCube, new Double[] { 11.226, 5.178, 0.0 });
        positionMap.put(ArmPosition.AboveMidConeTop, new Double[] { 1.547, -3.0, 0.2490 + absWristOffset });

        setArm(ArmPosition.StartingConfig);
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
        setElbow(elbow);;
        setWrist(wrist);
    };

    public void setArm(ArmPosition pos) {
        if (pos == ArmPosition.StartingConfig && targetPos != ArmPosition.StartingConfig) {
            lastMovement = Timer.getFPGATimestamp();
        }

        lastPose = targetPos;
        targetPos = pos;

        shoulderSpeed = (positionMap.get(targetPos)[0] - shoulderTarget);
        elbowSpeed = (positionMap.get(targetPos)[1] - elbowTarget);

        setShoulder(positionMap.get(targetPos)[0]);
        setElbow(positionMap.get(targetPos)[1]);
        // setWrist(positionMap.get(targetPos)[2]);

        // if(pos == ArmPosition.StartingConfig){
        //     setElbow(positionMap.get(targetPos)[1]);
        // }
        // else{
        //     setShoulder(positionMap.get(targetPos)[0]);
        // }
        setWrist(positionMap.get(targetPos)[2]);
    }

    @Override
    public void periodic() {
        
        double shoulderPower = 0, wristPower = 0, elbowPower = 0;
        double elbowAngle = (Math.PI/2) - ((Math.PI*2) - getElbowPos() + Math.toRadians(10) - getShoulderAngle());

        SmartDashboard.putNumber("elbow angle", elbowAngle);

        if (!controllerInterrupt) {             
            shoulderPower = pidShoulder.calculate(getShoulderPos(), shoulderTarget) + getShoulderFF();
           moveShoulder(shoulderPower);
            
            // pidShoulder.setReference(shoulderTarget, com.revrobotics.CANSparkMax.ControlType.kPosition);

            wristPower = pidWrist.calculate(getWristPos(), wristTarget);
           moveWrist(wristPower);

            elbowPower = pidElbow.calculate(getElbowPos(), elbowTarget) + getElbowFF(elbowAngle);
            moveElbow(elbowPower);

            // if(getElbowPos()<positionMap.get(targetPos)[1]+0.5 && getElbowPos()>positionMap.get(targetPos)[1]-0.5){
            //     setShoulder(positionMap.get(targetPos)[0]);
            // }
            // if(getShoulderPos()<positionMap.get(targetPos)[0]+0.5 && getShoulderPos()>positionMap.get(targetPos)[0]-0.5){
            //     setElbow(positionMap.get(targetPos)[1]); 
            // }

        }

        
    

        SmartDashboard.putNumber("Wrist power", wristPower);
        SmartDashboard.putNumber("Elbow power", elbowPower);
        SmartDashboard.putNumber("ElbowF FF", getElbowFF(elbowAngle));

        SmartDashboard.putNumber("Shoulder power", shoulderPower);

        SmartDashboard.putNumber("Shoulder position", getShoulderPos());
        SmartDashboard.putNumber("Elbow position", getElbowPos());
        // SmartDashboard.putNumber("Elbow FF", (Math.cos(elbowAngle) * ArmConstants.ELBOW_F));

        SmartDashboard.putNumber("Wrist position", getWristPos());

        // SmartDashboard.putNumber("Shoulder speed", shoulderSpeed);
        // SmartDashboard.putNumber("Shoulder target", shoulderTarget);
        // SmartDashboard.putNumber("Elbow angle", elbowAngle);
        // SmartDashboard.putNumber("Shoulder angle", getShoulderAngle());


        // SmartDashboard.putNumber("Elbow speed", elbowSpeed);


        // shoulderTarget = SmartDashboard.getNumber("Shoulder target",
        // getShoulderPos());
        elbowTarget = SmartDashboard.getNumber("Elbow target", getElbowPos());
        wristTarget = SmartDashboard.getNumber("Wrist target", getWristPos());
        double shoulderVal = 0, elbowVal = 0;

        shoulderVal = SmartDashboard.getNumber("Shoulder Change Margin", shoulderVal);
        elbowVal = SmartDashboard.getNumber("Elbow Change Margin", elbowVal);

        
    }

    public double getShoulderFF() {
        double shoulderPos = getShoulderPos();
        return (shoulderPos * 0.00296222) + 0.0237827;
    }

    public double getElbowFF(double angle) {
        return (double)(0.0927611 * Math.cos(angle));
    }

    public double getWristPos() {
        return wristEncoder.getPosition();
    }

    // Returns encoder count
    public double getShoulderPos() {
        return shoulderLeftEncoder.getPosition();
    }

    // Returns encoder count
    public double getElbowPos() {
        double pos = absElbowEncoder.getPosition();
        return (pos < 0.5) ? pos + (2*Math.PI) : pos;
    }

    // Returns shoulder angle (rad)
    public double getShoulderAngle() {
        return (shoulderLeftEncoder.getPosition() * ArmConstants.SHOULDER_ENCODER_TO_RAD) + ArmConstants.SHOULDER_START_ANGLE;
    }

    // Returns elbow angle (rad)
    public double getElbowAngle() {

        // String length (meters)
        double length = absElbowEncoder.getPosition() * ArmConstants.ELBOW_ENCODER_TO_METERS;

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

    public void moveShoulder(double power) {
        power = MathUtil.clamp(power, -0.25, 0.45);
        shoulderLeft.setVoltage(power * 12);
        shoulderRight.setVoltage(power * 12);
    }

    public void moveElbow(double power) {
        elbow.setVoltage(MathUtil.clamp(power, -0.15, 0.3) * 12);
    }

    public void breakZiptie() {
        elbow.set(-0.5);
    }

    public void moveWrist(double power) {
        power = MathUtil.clamp(power, -0.3, 0.3);
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
        wristEncoder.setPosition(0);
    }

    public boolean getLimitSwitch(){
        return limitSwitch.get();
    }
}