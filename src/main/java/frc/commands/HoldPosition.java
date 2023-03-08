package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class HoldPosition extends CommandBase {
    Arm arm;

    public HoldPosition(Arm arm) {
        this.arm = arm;
        
        addRequirements(arm);
    }

    public void initialize() {
        arm.controllerInterrupt = false;
        arm.setArm(arm.getShoulderPos(), arm.getElbowPos(), arm.getWristPos());
    }

    public boolean isFinished() {
        return true;
    }
}
