package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Intake;

public class RunIntakeCone extends CommandBase{
    Intake intake;

    public RunIntakeCone(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runIntakeCone();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }


}
