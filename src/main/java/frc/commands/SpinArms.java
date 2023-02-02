package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class SpinArms extends CommandBase{
    private Arm arm;

    public SpinArms(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.spinMotors(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        arm.spinMotors(0);
    }
}
