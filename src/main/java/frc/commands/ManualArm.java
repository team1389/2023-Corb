package frc.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;
import frc.subsystems.Arm.ArmPosition;

public class ManualArm extends CommandBase {
    Arm arm;
    Supplier<Double> shoulderFunction, elbowFunction;
    Supplier<Boolean> downWrist, upWrist, interrupt;
    double targetShoulder = 0;
    double targetElbow = 0;

    public ManualArm(Arm arm, Supplier<Double> shoulderFunction, Supplier<Double> elbowFunction, Supplier<Boolean> upWrist, Supplier<Boolean> downWrist, Supplier<Boolean> interrupt) {
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

        if ((Math.abs(shoulder) > 0.05 || Math.abs(elbow) > 0.05 || upWrist.get() || downWrist.get())
            && interrupt.get()) {

            arm.controllerInterrupt = true;
        }

        if (arm.controllerInterrupt) {
            arm.moveShoulder(MathUtil.clamp(shoulder * shoulder * shoulder, -1, 1));
            arm.moveElbow(MathUtil.clamp(elbow * elbow * elbow, -1, 1));
            if (upWrist.get()) {
                arm.moveWrist(0.4);
            }
            if (downWrist.get()) {
                arm.moveWrist(-0.4);
            }
            if(!upWrist.get() && !downWrist.get()) {
                arm.moveWrist(0);
            }
        }
    }

}
