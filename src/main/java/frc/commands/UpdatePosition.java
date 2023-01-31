package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Drivetrain;
import frc.subsystems.Vision;

public class UpdatePosition extends CommandBase{
    private final Drivetrain drivetrain;
    private final Vision vision;
    
    public UpdatePosition(Drivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(drivetrain, vision);
    }

    @Override
    public void execute() {
        vision.update(drivetrain);
        drivetrain.updateFieldPose();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
