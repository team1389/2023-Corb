package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Intake;

public class RunIntakeCube extends CommandBase{
    Intake intake;

    public RunIntakeCube(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runIntakeCube();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }


}
