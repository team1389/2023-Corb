package frc.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.AutoBalance;
import frc.commands.RunOuttake;
import frc.commands.SetArmPosition;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;

public class QuickBalance extends SequentialCommandGroup{
   
    public QuickBalance(Drivetrain drivetrain, Arm arm, Intake intake){
        addRequirements(drivetrain, arm, intake);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Quick Balance", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS));

        Command drivePath = drivetrain.followTrajectoryCommand(trajectory, true);

        addCommands(
            new SetArmPosition(arm, ArmPosition.High),
            new RunOuttake(intake),
            drivePath, 
            new AutoBalance(drivetrain)
        );
        
    }
}
