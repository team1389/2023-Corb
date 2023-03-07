package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;
import frc.subsystems.Arm.ArmPosition;

public class SetArmPosition extends CommandBase {
    private Arm arm;
    private ArmPosition target;
    private boolean runInstantly;

    public SetArmPosition(Arm arm, ArmPosition targetPosition, boolean runInstantly) {
        this.arm = arm;
        this.runInstantly = runInstantly;
        target = targetPosition;

        addRequirements(arm);
    }   

    @Override
    public void initialize() {
        arm.controllerInterrupt = false;
        arm.setArm(target);
    }

    @Override
    public boolean isFinished() {
        return runInstantly || arm.getAtPosition();
    }
}
