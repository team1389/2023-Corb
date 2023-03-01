package frc.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class ManualArm extends CommandBase {
    Arm arm;
    Supplier<Double> shoulderFunction, elbowFunction;

    public ManualArm(Arm arm, Supplier<Double> shoulderFunction, Supplier<Double> elbowFunction) {
        this.arm = arm;
        this.shoulderFunction = shoulderFunction;
        this.elbowFunction = elbowFunction;

        addRequirements(arm);
    }

    @Override
    public void execute() {

        var shoulder = shoulderFunction.get();
        var elbow = elbowFunction.get();
        if (Math.abs(shoulder) > 0.01 || Math.abs(elbow) > 0.01) {
            arm.controllerInterupt = true;
        }

        if (arm.controllerInterupt) {
            arm.moveShoulder(MathUtil.clamp(shoulder * shoulder * shoulder, -1, 1));
            arm.moveElbow(MathUtil.clamp(elbow * elbow * elbow, -1, 1));
        }
    }

}
