package frc.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Intake;

public class RunOuttakeCube extends CommandBase{
    Intake intake;
    double timeout = -1;
    Timer timer = new Timer();

    public RunOuttakeCube(Intake intake){
        this.intake = intake;
        timer.reset();
        timer.start();

        addRequirements(intake);
    }

    // timeout in seconds
    public RunOuttakeCube(Intake intake, double timeout){
        this.intake = intake;
        timer.reset();
        timer.start();
        this.timeout = timeout;

        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runOuttakeCube();
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        //this works i tihnk
        return timeout == -1 ^ timer.get() > timeout;
    }


}
