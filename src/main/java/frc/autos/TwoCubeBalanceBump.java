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
    List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("2 Cube Balance (bump)", new PathConstraints(
        AutoConstants.AUTO_MAX_METERS_PER_SEC, 
        AutoConstants.AUTO_MAX_MPSS)
    );

    Command drivePath = drivetrain.followTrajectoryCommand(trajectories.get(0), true);
    Command drivePath2 = drivetrain.followTrajectoryCommand(trajectories.get(1), false);

    addCommands(
        new SetArmPosition(arm, ArmPosition.HighCone, false, 2.1),
        new RunOuttakeCone(intake, 0.4),
        new SetArmPosition(arm, ArmPosition.StartingConfig, false, 0.8),
        new FollowPathWithEvents(drivePath, trajectories.get(0).getMarkers(), hmm),
        new RunOuttakeCube(intake, 0.5),
        new SetArmPosition(arm, ArmPosition.StartingConfig, true),
        drivePath2
    );
    }
}
