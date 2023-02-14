package frc.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class ManualArm extends CommandBase {
    Arm arm;
    Supplier<Double> shoulderFuction, elbowFunction;

    public ManualArm(Arm arm, Supplier<Double> shoulderFunction, Supplier<Double> elbowFunction) {
        this.arm = arm;
        this.shoulderFuction = shoulderFunction;
        this.elbowFunction = elbowFunction;

        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        arm.moveShoulder(MathUtil.clamp(shoulderFuction.get(), -0.25, 025));
        arm.moveElbow(MathUtil.clamp(elbowFunction.get(), -0.25, 025));
    }
}