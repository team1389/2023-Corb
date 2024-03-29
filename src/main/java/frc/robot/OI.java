package frc.robot;

import java.util.HashMap;


import frc.util.DPadButton;
import frc.util.DPadButton.Direction;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.autos.DriveBack;
import frc.autos.OneBottomCone;
import frc.autos.OneBottomCube;
import frc.autos.OutAndScoreNoBump;
import frc.autos.OverAndOut;
import frc.autos.PickupBalance;
import frc.autos.OneTopCube;
import frc.autos.OutAndScoreBump;
import frc.autos.QuickBalance;
import frc.autos.QuickBalanceCone;
import frc.autos.ThreeExclamation;
import frc.autos.TwoCubeBalanceBump;
import frc.autos.ConeCone;
import frc.autos.ConeCube;
import frc.autos.TwoTopCube;
import frc.autos.TwoTopCubeBalance;
import frc.commands.AutoAlign;
import frc.commands.HoldPosition;
import frc.commands.ManualArm;
import frc.commands.PickupCone;
import frc.commands.RunIntakeCone;
import frc.commands.RunIntakeCube;
import frc.commands.SetArmPosition;
import frc.commands.TeleOpDrive;
import frc.subsystems.Arm;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;
import frc.subsystems.Lights;
import frc.subsystems.Arm.ArmPosition;

public class OI {

    public final Drivetrain drivetrain = new Drivetrain();
    // public final Vision vision = new Vision();
    public final Intake intake = new Intake();
    public final Arm arm = new Arm();
    public final Lights light = new Lights();


    private GenericHID driveController;
    private Trigger driveRightBumper, driveLeftBumper;
    private Trigger driveAButton;
    private Trigger driveXButton;
    private Trigger driveBButton;
    private Trigger driveYButton;
    private Trigger driveRightTrigger;
    private Trigger driveLeftTrigger;



    private XboxController manipController;
    private Trigger manipEllipsisButton;
    private Trigger manipMenuButton;
    private Trigger manipFullscreen;
    private Trigger manipStadia;
    private Trigger manipGoogle;


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

    SendableChooser<Command> chooser = new SendableChooser<>();

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();

    public OI() {
    
        //automap
        autoMap.put("start intake", new InstantCommand(() -> intake.runIntakeCube()));
        autoMap.put("stop intake", new InstantCommand(() -> intake.stop()));
        autoMap.put("arm high cone", new SetArmPosition(arm, ArmPosition.HighCone, true));
        autoMap.put("arm cube intake", new SetArmPosition(arm, ArmPosition.IntakeCube, true));
        autoMap.put("arm cone intake", new SetArmPosition(arm, ArmPosition.IntakeCone, true));

        autoMap.put("arm mid cone", new SetArmPosition(arm, ArmPosition.MidCone, true));
        autoMap.put("arm starting", new SetArmPosition(arm, ArmPosition.StartingConfig, true));
        autoMap.put("arm high cube", new SetArmPosition(arm, ArmPosition.HighCube, true));
        autoMap.put("arm cube back", new SetArmPosition(arm, ArmPosition.CubeBack, true));

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
                () -> !getDriveRightBumper(), // Slow function
                () -> driveXButton.getAsBoolean(), // Hold x position
                () -> driveRightTrigger.getAsBoolean(),
                () -> driveController.getRawAxis(5)) // flip
        );

        // Press A button -> zero gyro heading
        driveAButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

        // Press X button -> set X to not slide
        driveXButton.onTrue(new InstantCommand(() -> drivetrain.setX()));

        driveBButton.toggleOnTrue(Commands.startEnd(() -> light.setColor(255,179,0),() -> light.setColor(104,0,142), light));
        driveYButton.onTrue(new InstantCommand(() -> {light.isRainbowing = true;}));

        // Auto Align with AprilTag
        driveLeftTrigger.onTrue(new AutoAlign(drivetrain));

        // MANIPULATOR CONTROLLER
        manipEllipsisButton.onTrue(new InstantCommand(() -> arm.resetEncoders()));
        manipStadia.onTrue(new SetArmPosition(arm, ArmPosition.StartingConfig, true));
        // manipFullscreen.onTrue(new SetArmPosition(arm, ArmPosition.IntakeConeFeeder, true));
        manipFullscreen.onTrue(new HoldPosition(arm));


        manipAButton.onTrue(new SetArmPosition(arm, ArmPosition.IntakeCone, true));
        manipXButton.onTrue(new SetArmPosition(arm, ArmPosition.MidCone, true));
        manipYButton.onTrue(new SetArmPosition(arm, ArmPosition.HighCone, true));

        manipRightTrigger.onTrue(new InstantCommand(() -> intake.shootCube()));
        manipRightTrigger.onFalse(new InstantCommand(() -> intake.stop()));

        manipDown.onTrue(new SetArmPosition(arm, ArmPosition.IntakeCube, true));
        manipUp.onTrue(new SetArmPosition(arm, ArmPosition.HighCube, true));
        manipLeft.onTrue(new SetArmPosition(arm, ArmPosition.MidCube, true));

        manipLeftTrigger.and(manipAButton).onTrue(new SetArmPosition(arm, ArmPosition.IntakeConeFeeder, true));
        manipLeftTrigger.and(manipXButton).onTrue(new SetArmPosition(arm, ArmPosition.MidConeBack, true));
        manipLeftTrigger.and(manipUp).onTrue(new SetArmPosition(arm, ArmPosition.CubeBack, true));
        manipLeftTrigger.and(manipYButton).onTrue(new SetArmPosition(arm, ArmPosition.AboveHighCone, true));

        manipGoogle.onTrue(new PickupCone(arm, intake, drivetrain));        

        manipBButton.whileTrue(new RunIntakeCone(intake));
        manipRight.whileTrue(new RunIntakeCube(intake));


        final Command oneBottomCone = new OneBottomCone(drivetrain, arm, intake, autoMap);
        final Command oneBottomCube = new OneBottomCube(drivetrain, arm, intake, autoMap);
        final Command oneTopCube = new OneTopCube(drivetrain, arm, intake, autoMap);
        final Command quickBalanceCube = new QuickBalance(drivetrain, arm, intake);
        final Command quickBalanceCone = new QuickBalanceCone(drivetrain, arm, intake, autoMap);
        final Command overAndOut = new OverAndOut(drivetrain, arm, intake);

        final Command driveBack = new DriveBack(drivetrain, arm, intake);
        final Command outAndScoreNoBump = new OutAndScoreNoBump(drivetrain, arm, intake, autoMap);
        final Command outAndScoreBump = new OutAndScoreBump(drivetrain, arm, intake);
        final Command twoTopCube = new TwoTopCube(drivetrain, arm, intake, autoMap);
        final Command twoTopCone = new ConeCube(drivetrain, arm, intake, autoMap);
        final Command twoTopCubeBalance = new TwoTopCubeBalance(drivetrain, arm, intake, autoMap);
        final Command twoCubeBalanceBump = new TwoCubeBalanceBump(drivetrain, arm, intake, autoMap);
        final Command threeExclamation = new ThreeExclamation(drivetrain, arm, intake, autoMap);
        final Command coneCone = new ConeCone(drivetrain, arm, intake, autoMap);
        final Command pickup = new PickupBalance(drivetrain, arm, intake, autoMap);





        // Add options
        chooser.addOption("Quick Balance Cube", quickBalanceCube);
        chooser.addOption("Quick Balance Cone", quickBalanceCone);

        chooser.addOption("Cone, over charge station, balance", overAndOut);
        chooser.addOption("Cone, pickup, cube but bump side", twoCubeBalanceBump);
        chooser.addOption("Cone, pickup, cube", twoTopCone);
        chooser.addOption("Cone, pickup, cone", coneCone);
        chooser.addOption("Over pickup score again balance", pickup);


        // chooser.addOption("Cube, pickup, cube, balance", twoTopCubeBalance);

        chooser.addOption("Cube, out, balance", outAndScoreNoBump);
        chooser.addOption("3!", threeExclamation);
        chooser.addOption("Drive Back", driveBack);


        SmartDashboard.putData("Auto choices", chooser);
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);

        driveAButton = new JoystickButton(driveController, 1);//X
        driveXButton = new JoystickButton(driveController, 3);//B
        driveBButton = new JoystickButton(driveController, 2);//A
        driveYButton = new JoystickButton(driveController, 4);

        driveRightBumper = new JoystickButton(driveController, 6);
        driveRightTrigger = new JoystickButton(driveController, 12);//right joystick btn

        driveLeftTrigger = new JoystickButton(driveController, 0); //put button number


        manipEllipsisButton = new JoystickButton(manipController, 9); //back btn
        manipMenuButton = new JoystickButton(manipController, 10); //start btn
        manipFullscreen = new JoystickButton(manipController, 15);//left Bumper - (15 (Stadia)) (5 - logitech)
        manipGoogle = new JoystickButton(manipController, 14);//right Bumper - (14 (Stadia)) (6 - logitech)
        manipStadia = new JoystickButton(manipController, 11);//left joystick btn

        manipRightBumper = new JoystickButton(manipController, XboxController.Button.kRightBumper.value);
        manipLeftBumper = new JoystickButton(manipController, XboxController.Button.kLeftBumper.value);
        manipRightTrigger = new JoystickButton(manipController, 12);//right joystick btn
        manipLeftTrigger = new JoystickButton(manipController, 13);//left Trigger - (13 (Stadia)) (7 - logitech)

        manipAButton = new JoystickButton(manipController, XboxController.Button.kA.value);//X
        manipBButton = new JoystickButton(manipController, XboxController.Button.kB.value);//A
        manipXButton = new JoystickButton(manipController, XboxController.Button.kX.value);//B
        manipYButton = new JoystickButton(manipController, XboxController.Button.kY.value);

        manipUp = new DPadButton(manipController, Direction.UP).getTrigger();
        manipLeft = new DPadButton(manipController, Direction.LEFT).getTrigger();
        manipDown = new DPadButton(manipController, Direction.DOWN).getTrigger();
        manipRight = new DPadButton(manipController, Direction.RIGHT).getTrigger();
    }

    private double getDriveLeftX() {
        return driveController.getRawAxis(0);
    }

    private double getDriveLeftY() {
        return driveController.getRawAxis(1);
    }

    private double getDriveRightX() {
        return driveController.getRawAxis(3); // logitech: 2, negative
    }

    private double getDriveRightY() {
        return driveController.getRawAxis(4); // logitech: 3
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
        return -manipController.getRawAxis(2);
    }

    private double getManipRightY() {
        return -manipController.getRawAxis(3);
    }

    private boolean getManipLeftBumper() {
        return !manipController.getRawButton(5);
    }

    private boolean getManipRightBumper() {
        return !manipController.getRawButton(6);
    }

    // Return auto command
    public Command getAutoCommand() {
        return chooser.getSelected();
    }

}