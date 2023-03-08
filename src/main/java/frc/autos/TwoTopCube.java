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
import frc.commands.RunOuttakeCube;
import frc.commands.SetArmPosition;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;

public class TwoTopCube extends SequentialCommandGroup{

    public TwoTopCube(Drivetrain drivetrain, Arm arm, Intake intake, HashMap<String, Command> hmm){
        
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 Top Cube", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC,
            AutoConstants.AUTO_MAX_MPSS)
        );

        Command path1 = drivetrain.followTrajectoryCommand(pathGroup.get(0), true);
        Command path2 = drivetrain.followTrajectoryCommand(pathGroup.get(1), false);

        // do stuff
        addCommands(
            new SetArmPosition(arm, ArmPosition.HighCube, false),
            new RunOuttakeCube(intake, 0.75),
            new SetArmPosition(arm, ArmPosition.Low, true),
            new FollowPathWithEvents(path1, pathGroup.get(0).getMarkers(), hmm),
            new RunOuttakeCube(intake, 0.75),
            new FollowPathWithEvents(path2, pathGroup.get(1).getMarkers(), hmm),
            new AutoBalance(drivetrain)
        );

    }
}
