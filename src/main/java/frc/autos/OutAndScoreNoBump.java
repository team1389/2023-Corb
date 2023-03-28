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
import frc.commands.RunOuttakeCube;
import frc.commands.SetArmPosition;
import frc.commands.TimeArm;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;
public class OutAndScoreNoBump extends SequentialCommandGroup{
    
    public OutAndScoreNoBump(Drivetrain drivetrain, Arm arm, Intake intake, HashMap<String, Command> hmm){
        
        addRequirements(drivetrain, arm, intake);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Out and Score (no bump)", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS));

        Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);

        addCommands(
            new SetArmPosition(arm, ArmPosition.HighCube, false, 1.5),
            new RunOuttakeCube(intake, 0.5),
            new SetArmPosition(arm, ArmPosition.StartingConfig, true),
            new FollowPathWithEvents(drivePath, trajectory.getMarkers(), hmm),
            new SetArmPosition(arm, ArmPosition.HighCube, false, 1.5),
            new RunOuttakeCube(intake, 0.5),
            new AutoBalance(drivetrain)
        );
    }
}
