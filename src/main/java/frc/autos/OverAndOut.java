package frc.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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

public class OverAndOut extends SequentialCommandGroup{
   
    public OverAndOut(Drivetrain drivetrain, Arm arm, Intake intake){
        addRequirements(drivetrain, arm, intake);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Over And Out", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC-2.121213, 
            AutoConstants.AUTO_MAX_MPSS-0.5));

        Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);

        addCommands(
            //new SetArmPosition(arm, ArmPosition.StartingConfig, false, 2),
            new SetArmPosition(arm, ArmPosition.HighCube, false, 1.5),
            new RunOuttakeCube(intake, 0.3),
            new SetArmPosition(arm, ArmPosition.StartingConfig, true),
            drivePath,
            new AutoBalance(drivetrain)
        );
        
    }
}
