package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Drivetrain;
import frc.subsystems.Vision;

public class AprilTagPoseEstimation extends CommandBase{
    private final Drivetrain drivetrain;
    private final Vision vision;
    
    public AprilTagPoseEstimation(Drivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        // TODO: This should not have a drivetrain requirement
        addRequirements(drivetrain, vision);
    }

    @Override
    public void execute() {
        vision.updatePose(drivetrain);
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
