package frc.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

public class OverAndOut extends SequentialCommandGroup{
   
    public OverAndOut(Drivetrain drivetrain, Arm arm, Intake intake){
        addRequirements(drivetrain, arm, intake);

        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("Over And Out 3", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC-2.7, 
            AutoConstants.AUTO_MAX_MPSS-0.9)
        );

        Command drivePath = drivetrain.followTrajectoryCommand(trajectories.get(0), true);

        addCommands(
            new SetArmPosition(arm, ArmPosition.HighCone, false, 2.1),
            new RunOuttakeCone(intake, 0.5),
            new SetArmPosition(arm, ArmPosition.StartingConfig, false, 0.5),
            drivePath,
            new AutoBalance(drivetrain)
        );
        
    }
}
