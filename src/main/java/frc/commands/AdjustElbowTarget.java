package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.*;

public class AdjustElbowTarget extends CommandBase{
    public Arm arm;
    private boolean inc;
    private double val;

    public AdjustElbowTarget(Arm arm, boolean increasing,double val) {
        this.arm=arm;
        inc = increasing;
        this.val = val;
    }    

    @Override
    public void initialize(){
        if(inc){
            arm.elbowTarget+=val;
        }
        else{
            arm.elbowTarget-=val;
        }

        //should be last
        arm.controllerInterrupt=false;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
