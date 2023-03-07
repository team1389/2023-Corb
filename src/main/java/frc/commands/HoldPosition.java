package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class HoldPosition extends CommandBase{
    Arm arm;
    
    public HoldPosition(Arm arm) {
        this.arm = arm;
    }

    public void initialize() {
        arm.controllerInterrupt = false;
        arm.shoulderTarget = arm.getShoulderPos();
        arm.elbowTarget = arm.getElbowPos();
        arm.wristTarget = arm.getWristPos();
    }

    public boolean isFinished() {
        return true;
    }
}
