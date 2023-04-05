package frc.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;

public class PickupCone extends CommandBase {
    private Intake intake;
    private Arm arm;
    private Drivetrain drivetrain;
    private double targetWristPos = 0.56;
    private double timeout = 0.9;
    private Timer timer = new Timer();


    public PickupCone(Arm arm, Intake intake, Drivetrain drivetrain) {
        this.intake = intake;
        this.arm = arm;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drivetrain.stopModules();
    }

    @Override
    public void execute() {
        if(arm.getWristPos() < targetWristPos){
            arm.setWrist(arm.wristTarget + 0.0225);
        } else {
            intake.runIntakeCone();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() > timeout;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}