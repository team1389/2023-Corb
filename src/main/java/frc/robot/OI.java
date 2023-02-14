package frc.robot;

import java.util.HashMap;

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
import frc.commands.AutoBalanceController;
import frc.commands.ManualArm;
import frc.commands.TeleOpDrive;
//import frc.commands.Test;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
//import frc.autos.TestAuto;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;

public class OI {

    public final Drivetrain drivetrain = new Drivetrain();
    public final Arm arm = new Arm();
    // public final Vision vision = new Vision();
    public final Intake intake = new Intake();

    private XboxController driveController;
    private Trigger driveRightBumper, driveLeftBumper;
    private Trigger driveAButton;

    private XboxController manipController;
    private Trigger manipAButton;
    private Trigger manipBButton;
    private Trigger manipXButton;
    private Trigger manipYButton;
    private Trigger manipLeftBumper;


    private HashMap<String, Command> hmmmmmmmmmmmmmm; 

    public OI() {
        hmmmmmmmmmmmmmm.put("start intake", new InstantCommand(() -> intake.runIntake()));
        hmmmmmmmmmmmmmm.put("stop intake", new InstantCommand(() -> intake.stop()));

        initControllers();
        
        // Cool new way to make a drive command by passing in Suppliers for the joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
            drivetrain,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> -driveController.getRightY(),
            () -> !driveController.getLeftBumper(),
            () -> driveController.getRightBumper()) // By default be in field oriented
        );

        arm.setDefaultCommand(new ManualArm(
            arm, 
            () -> manipController.getLeftX(), 
            () -> manipController.getRightX())
        );

        //drivetrain.setDefaultCommand(new AutoBalance(drivetrain));

        // Press right bumper -> zero gyro heading
        driveAButton.onTrue(new InstantCommand(()->drivetrain.zeroHeading()));
        // driveRightBumper.onTrue(new InstantCommand(()->drivetrain.zeroHeading()));

        // vision.setDefaultCommand(new AprilTagPoseEstimisation(drivetrain, vision));
        
        // manipAButton.onTrue(new RunIntake(intake));
        // manipBButton.onTrue(new RunOuttake(intake));
        // manipXButton.onTrue(new SetArm(arm, ArmPosition.Low));
        // manipYButton.onTrue(new SetArm(arm, ArmPosition.Mid));
        // manipLeftBumper.onTrue(new SetArm(arm, ArmPosition.High));
        //possibly add a wrist joint
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
        manipLeftBumper = new JoystickButton(manipController, XboxController.Button.kLeftBumper.value);
        manipBButton = new JoystickButton(manipController, XboxController.Button.kB.value);
        manipXButton = new JoystickButton(manipController, XboxController.Button.kX.value);
        manipYButton = new JoystickButton(manipController, XboxController.Button.kY.value);
    }

    // Return autocommand
    public Command getAutoCommand() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test Path", new PathConstraints(
            AutoConstants.AUTO_MAX_METERS_PER_SEC, 
            AutoConstants.AUTO_MAX_MPSS)
        );

        Command trajCommand = drivetrain.followTrajectoryCommand(trajectory, true);
        return trajCommand;
        //return new AutoBalanceController(drivetrain);
    }

}