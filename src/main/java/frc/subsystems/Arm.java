package frc.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ArmConstants;
import frc.robot.RobotMap.DriveConstants;

public class Arm extends SubsystemBase {
    private CANSparkMax shoulderLeft, shoulderRight, elbow, wrist;

    // private PIDController  pidShoulder;
    // private PIDController pidElbow;
    private PIDController pidWrist;

    private final TrapezoidProfile.Constraints backShoulderConstraints =
      new TrapezoidProfile.Constraints(90, 58);
    private final ProfiledPIDController pidShoulder =
      new ProfiledPIDController(RobotMap.ArmConstants.SHOULDER_P, RobotMap.ArmConstants.SHOULDER_I,
      RobotMap.ArmConstants.SHOULDER_D, backShoulderConstraints);

    private final TrapezoidProfile.Constraints backElbowConstraints =
      new TrapezoidProfile.Constraints(30, 14.6);
    private final ProfiledPIDController pidElbow =
      new ProfiledPIDController(RobotMap.ArmConstants.ELBOW_P, RobotMap.ArmConstants.ELBOW_I,
      RobotMap.ArmConstants.ELBOW_D, backElbowConstraints);
    
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
        AboveMidConeTop,
        MidConeBack,
        CubeBack
    }

    public Map<ArmPosition, Double[]> positionMap = new HashMap<ArmPosition, Double[]>();

    public ArmPosition targetPos = ArmPosition.StartingConfig;
    public ArmPosition lastPose = ArmPosition.StartingConfig;
    public double shoulderTarget;
    public double elbowTarget;
    public double wristTarget;
    private double elbowDelay = 0.0;
    private SparkMaxAbsoluteEncoder absElbowEncoder;
    private double absWristOffset = -0.005; // From 0.38
    // DigitalInput elbowLimitSwitch = new DigitalInput(0);
    // DigitalInput shoulderLimitSwitch = new DigitalInput(1);
    SparkMaxLimitSwitch wristLimitSwitch;



    // Number of steps to take to get there. Higher is smoother but slower

    public Arm() {

        shoulderLeft = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
        shoulderRight = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);

        elbow = new CANSparkMax(ArmConstants.ELBOW_MOTOR, MotorType.kBrushless);
        wrist = new CANSparkMax(DriveConstants.WRIST_MOTOR, MotorType.kBrushless);

        shoulderLeft.setIdleMode(IdleMode.kBrake);
        shoulderRight.setIdleMode(IdleMode.kBrake);
        elbow.setIdleMode(IdleMode.kBrake);
        wrist.setIdleMode(IdleMode.kBrake);

        shoulderLeft.setInverted(false);
        shoulderRight.setInverted(false);

        pidWrist = new PIDController(RobotMap.ArmConstants.WRIST_P, RobotMap.ArmConstants.WRIST_I,
                RobotMap.ArmConstants.WRIST_D);

        shoulderLeftEncoder = shoulderLeft.getEncoder();
        shoulderLeftEncoder.setPositionConversionFactor(1);
        shoulderLeftEncoder.setPosition(0);

        shoulderRightEncoder = shoulderLeft.getEncoder();
        wristEncoder = wrist.getEncoder();
        absElbowEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);


        // [Shoulder, elbow, wrist, delay for elbow going up, delay for elbow going down]
        // If elbow delay > 0, shoulder then elbow; if it's less, elbow then shoulder
        positionMap.put(ArmPosition.StartingConfig, new Double[] { 0.0, 2*Math.PI - 1.08, 0.0 });

        positionMap.put(ArmPosition.IntakeCube, new Double[] { 0.047619111, 3.85590171813, 0.0161137823 });
        positionMap.put(ArmPosition.IntakeCone, new Double[] { 1.5148, 3.95558656, 0.2025 });

        positionMap.put(ArmPosition.Low, new Double[] { 0.0, 0.0, 0.0 }); // TODO
        positionMap.put(ArmPosition.MidCone, new Double[] { 28.48322, 1.768219, 0.6075, 1.5, -1.85 });
        positionMap.put(ArmPosition.HighCone, new Double[] { 56.7858, 2.747688, 0.0, 9.0, 70.0 });
        positionMap.put(ArmPosition.IntakeConeFeeder, new Double[] { 10.674, 3.872696271218, 0.3825 });
        positionMap.put(ArmPosition.MidCube, new Double[] { 0.0, 4.8, 0.0 });
        positionMap.put(ArmPosition.HighCube, new Double[] { 6.569, 4.626, 0.0 });
        positionMap.put(ArmPosition.AboveMidConeTop, new Double[] { 1.547, -3.0, 0.2490 + absWristOffset });
        positionMap.put(ArmPosition.MidConeBack, new Double[] {50.15725, 4.0689259, 0.3375, 25.0});
        positionMap.put(ArmPosition.CubeBack, new Double[] {34.6110621, 5.2031853, 0.0});

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
        setElbow(elbow);
        setWrist(wrist);
    };

    public void setArm(ArmPosition pos) {
        if (pos == ArmPosition.StartingConfig && targetPos != ArmPosition.StartingConfig) {
            lastMovement = Timer.getFPGATimestamp();
        }

        lastPose = targetPos;
        targetPos = pos;

        setWrist(positionMap.get(targetPos)[2]);

        // Set elbow delay to 0 by default, or to the delay if applicable
        elbowDelay = 0.0;
        if(positionMap.get(pos).length > 3) {
            elbowDelay = positionMap.get(pos)[3];
        }
        if(pos == ArmPosition.StartingConfig && positionMap.get(lastPose).length > 4) {
            elbowDelay = positionMap.get(lastPose)[4];
        }

        // If elbow delay is positive, don't set elbow
        if(elbowDelay <= 0) {
            setElbow(positionMap.get(targetPos)[1]);
        }

        // If elbow delay is negative, don't set shoulder
        if (elbowDelay >= 0) {
            setShoulder(positionMap.get(targetPos)[0]);
        }

    }

    @Override
    public void periodic() {
        
        double shoulderPower = 0, wristPower = 0, elbowPower = 0;

        // Geometry?? I'd rather have an apple tree :3
        double elbowAngle = (Math.PI / 2) - ((Math.PI * 2) - getElbowPos() + Math.toRadians(25) - getShoulderAngle());
        // double elbowAngle = -((2 * Math.PI - getElbowPos() + Math.toRadians(25)) - getShoulderAngle()) + (Math.PI/2);

        SmartDashboard.putNumber("elbow angle", elbowAngle);
        SmartDashboard.putNumber("shoulder angle", getShoulderAngle());


        if (!controllerInterrupt) {             
            shoulderPower = pidShoulder.calculate(getShoulderPos(), shoulderTarget) + getShoulderFF();
            moveShoulder(shoulderPower);
            
            // pidShoulder.setReference(shoulderTarget, com.revrobotics.CANSparkMax.ControlType.kPosition);

            wristPower = pidWrist.calculate(getWristPos(), wristTarget);
            moveWrist(wristPower);

            elbowPower = pidElbow.calculate(getElbowPos(), elbowTarget);
            moveElbow(elbowPower);


            if(elbowDelay > 0 && getShoulderPos() > elbowDelay && targetPos != ArmPosition.StartingConfig) {
                setElbow(positionMap.get(targetPos)[1]);
            }
            else if(elbowDelay < 0 && getElbowPos() < -elbowDelay && targetPos != ArmPosition.StartingConfig) {
                setShoulder(positionMap.get(targetPos)[0]);
            }
            else if(elbowDelay > 0 && getShoulderPos() < elbowDelay && targetPos == ArmPosition.StartingConfig) {
                setElbow(positionMap.get(targetPos)[1]);
            }
            else if(elbowDelay < 0 && getElbowPos() > -elbowDelay && targetPos == ArmPosition.StartingConfig) {
                setShoulder(positionMap.get(targetPos)[0]);
            }
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
        // if(getShoulderLimitSwitch()){
        //     shoulderLeftEncoder.setPosition(0);
        //     shoulderRightEncoder.setPosition(0);
        //     if(power<0){
        //         return;
        //     }
        // }
        power = MathUtil.clamp(power, -0.6, 0.6);
        shoulderLeft.setVoltage(power * 12);
        shoulderRight.setVoltage(power * 12);
    }

    public void moveElbow(double power) {
        // if(getElbowLimitSwitch() && power>0){
        //     return;
        // }
        if(getElbowPos() > 5.24 && power > 0) {
            elbow.setVoltage(0);
            return;
        }
        elbow.setVoltage(MathUtil.clamp(power, -0.6, 0.6) * 12);
    }

    public void breakZiptie() {
        elbow.set(-0.5);
    }

    public void moveWrist(double power) {
        // if(getWristLimitSwitch()){
        //     wristEncoder.setPosition(0);
        //     if(power<0){
        //         return;
        //     }
        // }
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

    // public boolean getElbowLimitSwitch(){
    //     return elbowLimitSwitch.get();
    // }

    // public boolean getShoulderLimitSwitch(){
    //     return shoulderLimitSwitch.get();
    // }

    // public boolean getWristLimitSwitch(){
    //     return wristLimitSwitch.get();
    // }
}