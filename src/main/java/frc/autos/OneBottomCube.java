package frc.autos;
import java.util.HashMap;

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
public class OneBottomCube extends SequentialCommandGroup{
    
    public OneBottomCube(Drivetrain drivetrain, Arm arm, Intake intake, HashMap<String, Command> hmm){
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("1 Bottom Cube", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        
        Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);

        //score initial cube, pick up game piece, balance
        addCommands(
            new SetArmPosition(arm, ArmPosition.HighCube, false),
            new RunOuttakeCube(intake, 0.75),
            new SetArmPosition(arm, ArmPosition.Low, true),
            new FollowPathWithEvents(drivePath, trajectory.getMarkers(), hmm),
            new AutoBalance(drivetrain)
        );
    }
}
