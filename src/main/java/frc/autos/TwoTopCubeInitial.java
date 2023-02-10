package frc.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Drivetrain;

public class TwoTopCubeInitial extends SequentialCommandGroup{
    private final Drivetrain drivetrain;

    public TwoTopCubeInitial(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("2 Top Cube Initial", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS));

        Command trajCommand = drivetrain.followTrajectoryCommand(trajectory, true);
        addCommands(trajCommand);
        addRequirements(drivetrain);
    }
}
