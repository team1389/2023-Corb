package frc.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class TimeArm extends CommandBase {
    Arm arm;
    Supplier<Boolean> downWrist, upWrist, interrupt;
    double targetShoulder = 0;
    double targetElbow = 0;
    double timeout = 0.5;
    Timer timer = new Timer();

    public TimeArm(Arm arm) {
        this.arm = arm;

    }

    @Override
    public void execute() {

        arm.moveShoulder(-0.5);
        arm.moveElbow(0.5);
    }

    @Override
    public void initialize() {
        arm.controllerInterrupt = true;
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        arm.resetEncoders();
    }

    @Override 
    public boolean isFinished() {
        return timer.get() > timeout;
    }

}