package frc.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class ManualWrist extends CommandBase {
    Arm arm;
    //Supplier<Double> shoulderFuction, elbowFunction;
    private double power;

    public ManualWrist(Arm arm, double power) {
        this.arm = arm;
        // this.shoulderFuction = shoulderFunction;
        // this.elbowFunction = elbowFunction;, Supplier<Double> shoulderFunction, Supplier<Double> elbowFunction
        this.power = power;
        
        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        // arm.moveShoulder(MathUtil.clamp(shoulderFuction.get(), -0.25, 025));
        // arm.moveElbow(MathUtil.clamp(elbowFunction.get(), -0.25, 025));
        arm.moveWrist(power);
    }

    @Override
    public void end(boolean interrupted) {
        arm.moveWrist(0);
    }
}
