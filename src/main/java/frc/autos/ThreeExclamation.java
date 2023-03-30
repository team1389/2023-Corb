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

public class ThreeExclamation extends SequentialCommandGroup{
    public ThreeExclamation(Drivetrain drivetrain, Arm arm, Intake intake, HashMap<String, Command> hmm){
        
        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("3!", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );

        Command drivePath = drivetrain.followTrajectoryCommand(trajectories.get(0), true);
        Command drivePath2 = drivetrain.followTrajectoryCommand(trajectories.get(1), false);


        // do stuff
        addCommands(
            new SetArmPosition(arm, ArmPosition.HighCone, false, 2.11),
            new RunOuttakeCone(intake, 0.5),
            new SetArmPosition(arm, ArmPosition.StartingConfig, true),
            new FollowPathWithEvents(drivePath, trajectories.get(0).getMarkers(), hmm),
            new SetArmPosition(arm, ArmPosition.HighCube, false, 1.0),
            new RunOuttakeCube(intake, 0.5),
            new SetArmPosition(arm, ArmPosition.StartingConfig, true),
            new FollowPathWithEvents(drivePath2, trajectories.get(1).getMarkers(), hmm),
            new RunOuttakeCube(intake, 0.5)
        );

    } 
}
