package frc.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class BreakZiptie extends CommandBase {
    private Arm arm;
    double timeout = 0.412;
    Timer timer = new Timer();

    public BreakZiptie(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }   

    @Override
    public void initialize() {
        arm.controllerInterrupt = true;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        arm.breakZiptie();
    }

    @Override
    public boolean isFinished() {
        return  timer.get() > timeout;
    }

    @Override
    public void end(boolean interrupted) {
        arm.moveElbow(0);
        arm.controllerInterrupt = false;
    }
}
 