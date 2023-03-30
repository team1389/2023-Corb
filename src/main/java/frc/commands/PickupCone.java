package frc.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.subsystems.Drivetrain;

public class PickupCone extends ParallelCommandGroup {
    Drivetrain drivetrain;

    public PickupCone(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
}