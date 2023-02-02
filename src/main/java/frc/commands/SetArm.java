package frc.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;
import frc.subsystems.Arm.Position;

public class SetArm extends CommandBase{
    private Arm arm;
    private Position pos;     

    public SetArm(Arm arm, Position pos){
        this.arm = arm;
        this.pos = pos;
        addRequirements(arm);
    }

    @Override
    public void execute(){
        arm.setArm(pos);
    }
}
