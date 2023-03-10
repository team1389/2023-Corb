package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.util.DPadButton;
import frc.util.DPadButton.Direction;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.autos.OneBottomCone;
import frc.autos.OneBottomCube;
import frc.autos.OneTopCone;
import frc.autos.OneTopCube;
import frc.autos.QuickBalance;
import frc.autos.TwoTopCube;
import frc.commands.HoldPosition;
import frc.commands.ManualArm;
import frc.commands.ManualWrist;
import frc.commands.RunIntakeCone;
import frc.commands.RunIntakeCube;
import frc.commands.RunOuttakeCone;
import frc.commands.RunOuttakeCube;
import frc.commands.SetArmPosition;
import frc.commands.TeleOpDrive;
import frc.robot.RobotMap.AutoConstants;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Arm.ArmPosition;

public class OI {

    public final Drivetrain drivetrain = new Drivetrain();
    public final Arm arm = new Arm();
    // public final Vision vision = new Vision();
    public final Intake intake = new Intake();

    private GenericHID driveController;
    private Trigger driveRightBumper, driveLeftBumper;
    private Trigger driveAButton;
    private Trigger driveXButton;

    private XboxController manipController;
    private Trigger manipEllipsisButton;
    private Trigger manipMenuButton;
    private Trigger manipFullscreen;
    private Trigger manipStadia;

    private Trigger manipRightBumper;
    private Trigger manipLeftBumper;
    private Trigger manipRightTrigger;
    private Trigger manipLeftTrigger;

    private Trigger manipAButton;
    private Trigger manipBButton;
    private Trigger manipXButton;
    private Trigger manipYButton;

    private Trigger manipUp;
    private Trigger manipLeft;
    private Trigger manipDown;
    private Trigger manipRight;
    // private Trigger manipTopRight;
    // private Trigger manipBottomLeft;
    // private Trigger manipBottomRight;

    SendableChooser<Command> chooser = new SendableChooser<>();

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();

    public OI() {
        autoMap.put("start intake", new InstantCommand(() -> intake.runIntakeCube()));
        autoMap.put("stop intake", new InstantCommand(() -> intake.stop()));
        autoMap.put("arm high cone", new SetArmPosition(arm, ArmPosition.HighConeTop, true));
        autoMap.put("arm cube intake", new SetArmPosition(arm, ArmPosition.IntakeCube, true));
        autoMap.put("arm mid cone", new SetArmPosition(arm, ArmPosition.MidConeTop, true));
        autoMap.put("arm starting", new SetArmPosition(arm, ArmPosition.StartingConfig, true));

        initControllers();

        arm.setDefaultCommand(new ManualArm(
            arm,
            () -> getManipLeftY(),
            () -> getManipRightY(),
            () -> manipLeftBumper.getAsBoolean(),
            () -> manipRightBumper.getAsBoolean(),
            () -> manipMenuButton.getAsBoolean()
            )
        
        );

        // DRIVE CONTROLLER

        // Cool new way to make a drive command by passing in Suppliers for the
        // joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
                drivetrain,
                () -> getDriveLeftY(),
                () -> getDriveLeftX(),
                () -> getDriveRightX(),
                () -> getDriveRightY(),
                () -> getDriveLeftBumper(), // By default be in field oriented
                () -> !getDriveRightBumper()) // Slow function
        );

        // Press A button -> zero gyro heading
        driveAButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

        // Press X button -> set X to not slide
        driveXButton.onTrue(new InstantCommand(() -> drivetrain.setX()));

        // MANIPULATOR CONTROLLER
        // manipLeftBumper.whileTrue(new ManualWrist(arm, 0.4));
        // manipRightBumper.whileTrue(new ManualWrist(arm, -0.4));

        manipEllipsisButton.onTrue(new InstantCommand(() -> arm.resetEncoders()));

        manipStadia.onTrue(new SetArmPosition(arm, ArmPosition.StartingConfig, true));

        manipRightTrigger.and(manipAButton).onTrue(new SetArmPosition(arm, ArmPosition.IntakeConeTop, true));
        manipRightTrigger.and(manipXButton).onTrue(new SetArmPosition(arm, ArmPosition.MidConeTop, true));
        manipRightTrigger.and(manipYButton).onTrue(new SetArmPosition(arm, ArmPosition.HighConeTop, true));

        manipLeftTrigger.and(manipAButton).onTrue(new SetArmPosition(arm, ArmPosition.IntakeConeBottom, true));
        manipLeftTrigger.and(manipXButton).onTrue(new SetArmPosition(arm, ArmPosition.MidConeBottom, true));
        manipLeftTrigger.and(manipYButton).onTrue(new SetArmPosition(arm, ArmPosition.HighConeBottom, true));

        manipDown.onTrue(new SetArmPosition(arm, ArmPosition.IntakeCube, true));
        manipUp.onTrue(new SetArmPosition(arm, ArmPosition.HighCube, true));
        manipLeft.onTrue(new SetArmPosition(arm, ArmPosition.MidCube, true));

        // TODO: these are terrible names, perhaps "Backwards" and "Forwards"?
        manipBButton.whileTrue(new RunIntakeCone(intake));
        manipRight.whileTrue(new RunIntakeCube(intake));

        final Command oneBottomCone = new OneBottomCone(drivetrain, arm, intake, autoMap);
        final Command oneTopCone = new OneTopCone(drivetrain, arm, intake, autoMap);
        final Command oneBottomCube = new OneBottomCube(drivetrain, arm, intake, autoMap);
        final Command oneTopCube = new OneTopCube(drivetrain, arm, intake, autoMap);
        final Command quickBalance = new QuickBalance(drivetrain, arm, intake);
        final Command twoTopCube = new TwoTopCube(drivetrain, arm, intake, autoMap);

        // A chooser for autonomous commands

        chooser.setDefaultOption("One Bottom Cone", oneBottomCone);
        chooser.addOption("One Bottom Cube", oneBottomCube);
        chooser.addOption("One Top Cone", oneTopCone);
        chooser.addOption("One Top Cube", oneTopCube);
        chooser.addOption("Quick Balance", quickBalance);
        chooser.addOption("Two Top Cube", twoTopCube);
        SmartDashboard.putData("Auto choices", chooser);
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);

        driveAButton = new JoystickButton(driveController, 1);
        driveXButton = new JoystickButton(driveController, 3);
        driveRightBumper = new JoystickButton(driveController, 6);

        manipEllipsisButton = new JoystickButton(manipController, 9);
        manipMenuButton = new JoystickButton(manipController, 10);
        manipFullscreen = new JoystickButton(manipController, 15);
        manipStadia = new JoystickButton(manipController, 11);

        manipRightBumper = new JoystickButton(manipController, XboxController.Button.kRightBumper.value);
        manipLeftBumper = new JoystickButton(manipController, XboxController.Button.kLeftBumper.value);
        manipRightTrigger = new JoystickButton(manipController, 12);
        manipLeftTrigger = new JoystickButton(manipController, 13);

        manipAButton = new JoystickButton(manipController, XboxController.Button.kA.value);
        manipBButton = new JoystickButton(manipController, XboxController.Button.kB.value);
        manipXButton = new JoystickButton(manipController, XboxController.Button.kX.value);
        manipYButton = new JoystickButton(manipController, XboxController.Button.kY.value);

        manipUp = new DPadButton(manipController, Direction.UP).getTrigger();
        manipLeft = new DPadButton(manipController, Direction.LEFT).getTrigger();
        manipDown = new DPadButton(manipController, Direction.DOWN).getTrigger();
        manipRight = new DPadButton(manipController, Direction.RIGHT).getTrigger();
    }

    private double getDriveLeftX() {
        return -driveController.getRawAxis(0);
    }

    private double getDriveLeftY() {
        return -driveController.getRawAxis(1);
    }

    private double getDriveRightX() {
        return -driveController.getRawAxis(3);
    }

    private double getDriveRightY() {
        return driveController.getRawAxis(4);
    }

    private boolean getDriveLeftBumper() {
        return !driveController.getRawButton(5);
    }

    private boolean getDriveRightBumper() {
        return !driveController.getRawButton(6);
    }

    private double getManipLeftX() {
        return -manipController.getRawAxis(0);
    }

    private double getManipLeftY() {
        return -manipController.getRawAxis(1);
    }

    private double getManipRightX() {
        return -manipController.getRawAxis(3);
    }

    private double getManipRightY() {
        return -manipController.getRawAxis(4);
    }

    private boolean getManipLeftBumper() {
        return !manipController.getRawButton(5);
    }

    private boolean getManipRightBumper() {
        return !manipController.getRawButton(6);
    }

    // Return autocommand
    public Command getAutoCommand() {
        // PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test Path", new
        // PathConstraints(
        // AutoConstants.AUTO_MAX_METERS_PER_SEC,
        // AutoConstants.AUTO_MAX_MPSS)
        // );

        // Command trajCommand = drivetrain.followTrajectoryCommand(trajectory, true);
        // return trajCommand;
        // return new AutoBalanceController(drivetrain);
        
        // return chooser.getSelected();
        return new OneBottomCube(drivetrain, arm, intake, autoMap);
    }

}