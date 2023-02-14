package frc.commands;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.subsystems.Drivetrain;

public class Pickup extends ParallelCommandGroup{
    public Pickup(Drivetrain drivetrain) {
    }
}
