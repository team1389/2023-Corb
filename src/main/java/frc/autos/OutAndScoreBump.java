package frc.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.AutoBalance;
import frc.commands.RunOuttakeCone;
import frc.commands.SetArmPosition;
import frc.commands.TimeArm;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;
public class OutAndScoreBump extends SequentialCommandGroup{
    
    public OutAndScoreBump(Drivetrain drivetrain, Arm arm, Intake intake){
        
        addRequirements(drivetrain, arm, intake);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Out and Score (bump)", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS));

        PathPlannerTrajectory driveUpTraj = PathPlanner.loadPath("Drive Up", new PathConstraints(
                AutoConstants.AUTO_MAX_METERS_PER_SEC, 
                AutoConstants.AUTO_MAX_MPSS));

        Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);
        Command driveUp = drivetrain.followTrajectoryCommand(driveUpTraj, true);

        addCommands(
            //new SetArmPosition(arm, ArmPosition.StartingConfig, false, 2),
            new TimeArm(arm),
            new SetArmPosition(arm, ArmPosition.MidConeTop, false, 2.2),
            driveUp,
            new RunOuttakeCone(intake, 0.5),
            new SetArmPosition(arm, ArmPosition.StartingConfig, true),
            drivePath,
            new AutoBalance(drivetrain)
        );
    }
}
