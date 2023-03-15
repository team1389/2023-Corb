package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.*;

public class AdjustShoulderTarget extends CommandBase{
    public Arm arm;
    private boolean inc;
    private double val;

    public AdjustShoulderTarget(Arm arm, boolean increasing,double val) {
        this.arm=arm;
        inc = increasing;
        this.val = val;
    }    

    @Override
    public void initialize(){
        if(inc){
            arm.shoulderTarget+=val;
        }
        else{
            arm.shoulderTarget-=val;
        }

        //should be last
        arm.controllerInterrupt=false;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
