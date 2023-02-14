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
public class OneBottomCone extends SequentialCommandGroup{
    
    public OneBottomCone(Drivetrain drivetrain, Arm arm, Intake intake){
        
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("1 Bottom Cone Balance", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("1 Bottom Cone Initial", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );
        //TODO: Add command to start and stop intake
        //TODO: add auto for auto balancing
        //scores initial cone, pick up cone, balance
        
        addCommands(
            new SetArmPosition(arm, ArmPosition.High),
            new RunOuttake(intake),
            new SetArmPosition(arm, ArmPosition.Low),
            drivetrain.followTrajectoryCommand(trajectory1, true),
            //pick up
            drivetrain.followTrajectoryCommand(trajectory2, false),
            new AutoBalance(drivetrain)
        );
        addRequirements(drivetrain, arm, intake);
    }
}
