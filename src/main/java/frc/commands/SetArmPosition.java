package frc.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;
import frc.subsystems.Arm.ArmPosition;

public class SetArmPosition extends CommandBase {
    Arm arm;
    ArmPosition target;

    public SetArmPosition(Arm arm, ArmPosition targetPosition) {
        this.arm = arm;
        target = targetPosition;

        addRequirements(arm);
    }   

    @Override
    public void initialize() {
        arm.setArm(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
