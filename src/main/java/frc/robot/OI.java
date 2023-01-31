package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.commands.AutoBalance;
import frc.commands.RunIntake;
import frc.commands.RunOuttake;
import frc.commands.TeleOpDrive;
import frc.commands.UpdatePosition;
//import frc.commands.Test;
import frc.robot.RobotMap.AutoConstants;
//import frc.autos.TestAuto;
import frc.commands.UpdatePosition;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Vision;

public class OI {

    public final Drivetrain drivetrain = new Drivetrain();
    public final Vision vision = new Vision();
    public final Intake intake = new Intake();

    private XboxController driveController;
    private Trigger driveRightBumper;
    private Trigger driveAButton;

    private XboxController manipController;
    private Trigger manipAButton;
    private Trigger manipBButton;

    public OI() {
        initControllers();
        
        // Cool new way to make a drive command by passing in Suppliers for the joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
            drivetrain,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> -driveController.getRightY(),
            () -> !driveController.getLeftBumper()) // By default be in field oriented
        );
        //drivetrain.setDefaultCommand(new AutoBalance(drivetrain));

        // Press right bumper -> zero gyro heading
        driveRightBumper.onTrue(new InstantCommand(()->drivetrain.zeroHeading()));

        vision.setDefaultCommand(new UpdatePosition(drivetrain, vision));
        
        manipAButton.onTrue(new RunIntake(intake));
        manipBButton.onTrue(new RunOuttake(intake));
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        driveRightBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
        driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value);

        manipController = new XboxController(1);
        manipAButton = new JoystickButton(manipController, XboxController.Button.kA.value);
        manipBButton = new JoystickButton(manipController, XboxController.Button.kB.value);
    }

    // Return autocommand
    public Command getAutoCommand() {
        // PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test Path", new PathConstraints(
        //     AutoConstants.AUTO_MAX_METERS_PER_SEC, 
        //     AutoConstants.AUTO_MAX_MPSS)
        // );

        // Command trajCommand = drivetrain.followTrajectoryCommand(trajectory, true);
        // return trajCommand;
        return new AutoBalance(drivetrain);
    }

}