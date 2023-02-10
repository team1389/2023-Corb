package frc.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Drivetrain;

public class Test extends SequentialCommandGroup{
    Drivetrain drivetrain;
    
    public Test(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        
        PathPlannerTrajectory forwardTrajectory = PathPlanner.loadPath("2 meter forward", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        PathPlannerTrajectory leftTrajectory = PathPlanner.loadPath("2 meter left", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );

        Command forwardCommand = drivetrain.followTrajectoryCommand(forwardTrajectory, true);
        Command leftCommand = drivetrain.followTrajectoryCommand(leftTrajectory, false);
        
        addCommands(forwardCommand, leftCommand);
        addRequirements(drivetrain);
    }
}
