package frc.autos;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.AutoBalance;
import frc.commands.RunOuttakeCone;
import frc.commands.SetArmPosition;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;
public class OneTopCone extends SequentialCommandGroup{
    
    public OneTopCone(Drivetrain drivetrain, Arm arm, Intake intake, HashMap<String, Command> hmm){
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("1 Top Cone", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        
        Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);

        //score initial cone, pick up game piece, balance
        addCommands(
            // new SetArmPosition(arm, ArmPosition.HighCone, false),
            new RunOuttakeCone(intake),
            new SetArmPosition(arm, ArmPosition.Low, true),
            new FollowPathWithEvents(drivePath, trajectory.getMarkers(), hmm),
            new AutoBalance(drivetrain)
        );
    }
}
