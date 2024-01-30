package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Drivetrain;

public class AutoTarget extends CommandBase{
    private final Drivetrain drivetrain;
 //  PIDController pid = new PIDController(kP, kI, kD);
    private double yaw = SmartDashboard.getNumber("yaw", 0);

    public AutoTarget(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){

    }

    // @Override
    // public boolean isFinished() {
        
    // }
}
