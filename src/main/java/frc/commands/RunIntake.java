package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Intake;

public class RunIntake extends CommandBase{
    private Intake intake;

    public RunIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void execute(){
        intake.runIntake();
    }
    
    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

}

