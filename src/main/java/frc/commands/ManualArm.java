package frc.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class ManualArm extends CommandBase {
    Arm arm;
    Supplier<Double> shoulderFunction, elbowFunction;
    Supplier<Boolean> downWrist, upWrist, interrupt;
    double targetShoulder = 0;
    double targetElbow = 0;

    public ManualArm(Arm arm, Supplier<Double> shoulderFunction, Supplier<Double> elbowFunction,
            Supplier<Boolean> upWrist, Supplier<Boolean> downWrist, Supplier<Boolean> interrupt) {
        this.arm = arm;
        this.shoulderFunction = shoulderFunction;
        this.elbowFunction = elbowFunction;
        this.downWrist = downWrist;
        this.upWrist = upWrist;
        this.interrupt = interrupt;

        addRequirements(arm);
    }

    @Override
    public void execute() {

        double shoulder = shoulderFunction.get();
        double elbow = elbowFunction.get();
        // Check whether to interrupt PID loops
        if (interrupt.get()) {
            arm.controllerInterrupt = true;
        }

        var sho = shoulder;
        double elb = elbow;
        if (arm.controllerInterrupt) {
            // Cube speeds for easier control
            arm.moveShoulder(MathUtil.clamp(sho, -1, 1));

            arm.moveElbow(MathUtil.clamp(elb, -1, 1));

            if (upWrist.get()) {
                arm.moveWrist(0.2);
            } else if (downWrist.get()) {
                arm.moveWrist(-0.2);
            } else if (!upWrist.get() && !downWrist.get()) {
                arm.moveWrist(0);
            }
        } else {
            arm.setShoulder(arm.shoulderTarget + sho * 0.025*7);
            arm.setElbow(arm.elbowTarget + elb * 0.0225);
            if (upWrist.get()) {
                arm.setWrist(MathUtil.clamp(arm.wristTarget + 0.0225, -10000, 10000.585));
            } else if (downWrist.get()) {
                arm.setWrist(MathUtil.clamp(arm.wristTarget - 0.0225, -1000, 10000.585));
            } 

        }
    }

}
