package frc.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.subsystems.Drivetrain;

public class AutoAlign extends CommandBase{
    private final Drivetrain drivetrain;

    public AutoAlign(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){

        SmartDashboard.putNumber("LimelightX", LimelightHelpers.getTX(""));
        SmartDashboard.putString("InsideAutoAlign", "success");
        SmartDashboard.putNumber("LimelightY", LimelightHelpers.getTY(""));
        SmartDashboard.putNumber("LimelightArea", LimelightHelpers.getTA(""));
    }
}
