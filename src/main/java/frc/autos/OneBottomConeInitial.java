package frc.autos;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Drivetrain;
public class OneBottomConeInitial extends SequentialCommandGroup{
    Drivetrain drivetrain;
    
    public OneBottomConeInitial(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("1 Bottom Cone Initial", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        //TODO: Add command to start and stop intake
        addCommands(drivetrain.followTrajectoryCommand(trajectory, true));
        addRequirements(drivetrain);
    }
}
