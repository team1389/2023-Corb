package frc.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.AutoBalance;
import frc.commands.RunOuttake;
import frc.commands.SetArmPosition;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;

public class TwoTopCubeInitial extends SequentialCommandGroup{

    public TwoTopCubeInitial(Drivetrain drivetrain, Arm arm, Intake intake){
        addRequirements(drivetrain);
        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("2 Top Cube Initial", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("2 Top Cube 2nd Score", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("2 Top Cube Balance", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );


        addCommands(
            new SetArmPosition(arm, ArmPosition.High),
            new RunOuttake(intake),
            new SetArmPosition(arm, ArmPosition.Low),
            drivetrain.followTrajectoryCommand(trajectory1, true),
            //pick up
            drivetrain.followTrajectoryCommand(trajectory2, false),
            new SetArmPosition(arm, ArmPosition.High),
            new RunOuttake(intake),
            drivetrain.followTrajectoryCommand(trajectory3, false),
            new AutoBalance(drivetrain)
        );

    }
}
