package frc.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.commands.AutoBalance;
import frc.commands.PickupCone;
import frc.commands.RunOuttakeCone;
import frc.commands.RunOuttakeCube;
import frc.commands.SetArmPosition;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Arm.ArmPosition;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;

public class PickupBalance extends SequentialCommandGroup{

    public PickupBalance(Drivetrain drivetrain, Arm arm, Intake intake, HashMap<String, Command> hmm){
        
        PathPlannerTrajectory traj1 = PathPlanner.loadPath("Pickup Balance 1", new PathConstraints(
                AutoConstants.AUTO_MAX_METERS_PER_SEC-2.7, 
                AutoConstants.AUTO_MAX_MPSS-0.9));

        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("Pickup Balance 2", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );

        PathPlannerTrajectory traj3 = PathPlanner.loadPath("Pickup Balance 2", new PathConstraints(
                AutoConstants.AUTO_MAX_METERS_PER_SEC-2.7, 
                AutoConstants.AUTO_MAX_MPSS-0.9));

        Command drivePath1 = drivetrain.followTrajectoryCommand(traj1, true);
        Command drivePath2 = drivetrain.followTrajectoryCommand(trajectories.get(0), false);
        Command drivePath3 = drivetrain.followTrajectoryCommand(trajectories.get(1), false);
        Command drivePath4 = drivetrain.followTrajectoryCommand(traj3, false);




        // do stuff
        addCommands(
            new SetArmPosition(arm, ArmPosition.HighCone, false, 2.1),
            new RunOuttakeCone(intake, 0.5),
            new SetArmPosition(arm, ArmPosition.StartingConfig, false, 0.5),
            drivePath1,
            new SetArmPosition(arm, ArmPosition.IntakeCube, true),
            new FollowPathWithEvents(drivePath2, trajectories.get(0).getMarkers(), hmm),
            new FollowPathWithEvents(drivePath3, trajectories.get(1).getMarkers(), hmm),
            drivePath4,
            new ParallelCommandGroup(
                new AutoBalance(drivetrain),
                new SequentialCommandGroup(new WaitCommand(0.3),
                    new SetArmPosition(arm, ArmPosition.HighCube, false, 0.5),
                    new RunOuttakeCube(intake, 2)))
        );

    }
}
 