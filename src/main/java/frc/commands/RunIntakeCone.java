package frc.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Intake;

public class RunIntakeCone extends CommandBase {
    Intake intake;
    double timeout = -1;
    Timer timer = new Timer();

    public RunIntakeCone(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    // timeout in seconds
    public RunIntakeCone(Intake intake, double timeout) {
        this.intake = intake;
        this.timeout = timeout;

        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intake.runIntakeCone();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        // this works i tihnk
        return timeout == -1 ^ timer.get() > timeout;
    }

}
