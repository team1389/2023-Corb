package frc.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.AutoBalance;
import frc.commands.RunOuttakeCone;
import frc.commands.RunOuttakeCube;
import frc.commands.SetArmPosition;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;

public class TwoCubeBalanceBump extends SequentialCommandGroup{
    public TwoCubeBalanceBump(Drivetrain drivetrain, Arm arm, Intake intake, HashMap<String, Command> hmm){
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("2 Cube Balance (bump)", new PathConstraints(
        AutoConstants.AUTO_MAX_METERS_PER_SEC+2, 
        AutoConstants.AUTO_MAX_MPSS+0.67)
    );
    
    Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);

    addCommands(
        new SetArmPosition(arm, ArmPosition.HighCone, false, 2.1),
        new RunOuttakeCone(intake, 0.4),
        new SetArmPosition(arm, ArmPosition.StartingConfig, false, 0.8),
        new FollowPathWithEvents(drivePath, trajectory.getMarkers(), hmm),
        new RunOuttakeCube(intake, 0.5),
        new AutoBalance(drivetrain)
    );
    }
}
