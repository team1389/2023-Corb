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

public class QuickBalance extends SequentialCommandGroup{
   
    public QuickBalance(Drivetrain drivetrain, Arm arm, Intake intake){
        addRequirements(drivetrain, arm, intake);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Quick Balance", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC-3.0, 
            AutoConstants.AUTO_MAX_MPSS-1.25));

        PathPlannerTrajectory driveUpTraj = PathPlanner.loadPath("Drive Up", new PathConstraints(
                AutoConstants.AUTO_MAX_METERS_PER_SEC, 
                AutoConstants.AUTO_MAX_MPSS));

        Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);
        Command driveUp = drivetrain.followTrajectoryCommand(driveUpTraj, true);

        addCommands(
            new SetArmPosition(arm, ArmPosition.HighCube, false, 1.9),
            new RunOuttakeCube(intake, 0.5),
            new SetArmPosition(arm, ArmPosition.StartingConfig, false, 0.5),
            drivePath,
            new AutoBalance(drivetrain)
        );
        
    }
}
