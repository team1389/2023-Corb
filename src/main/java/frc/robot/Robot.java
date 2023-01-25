package frc.robot;


import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.SwerveTelemetry;

/**
 * Don't change the name of this class since the VM is set up to run this
 */
public class Robot extends TimedRobot {


    private OI oi;
    private Command autoCommand;
    SwerveTelemetry frontLeftTelemetry;
    SwerveTelemetry backLeftTelemetry;
    SwerveTelemetry frontRightTelemetry;
    SwerveTelemetry backRightTelemetry;
    PowerDistribution pdh;


    @Override
    public void robotInit() {
        
        oi = new OI();

        frontLeftTelemetry = new SwerveTelemetry(oi.drivetrain.frontLeft);
        backLeftTelemetry = new SwerveTelemetry(oi.drivetrain.backLeft);
        frontRightTelemetry = new SwerveTelemetry(oi.drivetrain.frontRight);
        backRightTelemetry = new SwerveTelemetry(oi.drivetrain.backRight);

        pdh = new PowerDistribution();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        double voltage = pdh.getVoltage();
        SmartDashboard.putNumber("Voltage", voltage);

        SmartDashboard.putNumber("FR Drive Current", pdh.getCurrent(RobotMap.DriveConstants.FR_DRIVE_PORT));
        SmartDashboard.putNumber("FL Drive Current", pdh.getCurrent(RobotMap.DriveConstants.FL_DRIVE_PORT));
        SmartDashboard.putNumber("BR Drive Current", pdh.getCurrent(RobotMap.DriveConstants.BR_DRIVE_PORT));
        SmartDashboard.putNumber("BL Drive Current", pdh.getCurrent(RobotMap.DriveConstants.BL_DRIVE_PORT));
        SmartDashboard.putNumber("FR Turn Current", pdh.getCurrent(RobotMap.DriveConstants.FR_TURN_PORT));
        SmartDashboard.putNumber("FL Turn Current", pdh.getCurrent(RobotMap.DriveConstants.FL_TURN_PORT));
        SmartDashboard.putNumber("BL Turn Current", pdh.getCurrent(RobotMap.DriveConstants.BL_TURN_PORT));
        SmartDashboard.putNumber("BR Turn Current", pdh.getCurrent(RobotMap.DriveConstants.BR_TURN_PORT));

    }


    @Override
    public void autonomousInit() {
        // Setting auto
        autoCommand = oi.getAutoCommand();

        if(autoCommand != null) {
            autoCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // SwerveTelemetry frontLeftTelemetry = new SwerveTelemetry(oi.drivetrain.frontLeft);
        // SwerveTelemetry backLeftTelemetry = new SwerveTelemetry(oi.drivetrain.backLeft);
        // SwerveTelemetry frontRightTelemetry = new SwerveTelemetry(oi.drivetrain.frontRight);
        // SwerveTelemetry backRightTelemetry = new SwerveTelemetry(oi.drivetrain.backRight);
        // //SendableRegistry.add(frontLeftTelemetry, "Swerve");

        // //SmartDashboard.putNumber("FL angular", frontLeftTelemetry.get);
       
        // SendableRegistry.addLW(frontLeftTelemetry, "FL Swerve");
        // SendableRegistry.addLW(backLeftTelemetry, "BL Swerve");
        // SendableRegistry.addLW(frontRightTelemetry, "FR Swerve");
        // SendableRegistry.addLW(backRightTelemetry, "BR Swerve");

        oi.drivetrain.frontLeft.resetEncoders();
        oi.drivetrain.backLeft.resetEncoders();
        oi.drivetrain.frontRight.resetEncoders();
        oi.drivetrain.backRight.resetEncoders();
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("FL angle", frontLeftTelemetry.getAngle());
        SmartDashboard.putNumber("FL abs", frontLeftTelemetry.getAbsAngle());

        SmartDashboard.putNumber("BL angle", backLeftTelemetry.getAngle());
        SmartDashboard.putNumber("BL abs", backLeftTelemetry.getAbsAngle());

        SmartDashboard.putNumber("FR angle", frontRightTelemetry.getAngle());
        SmartDashboard.putNumber("FR abs", frontRightTelemetry.getAbsAngle());

        SmartDashboard.putNumber("BR angle", backRightTelemetry.getAngle());
        SmartDashboard.putNumber("BR abs", backRightTelemetry.getAbsAngle());

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testInit() {
        SwerveTelemetry frontLeftTelemetry = new SwerveTelemetry(oi.drivetrain.frontLeft);
        SwerveTelemetry backLeftTelemetry = new SwerveTelemetry(oi.drivetrain.backLeft);
        SwerveTelemetry frontRightTelemetry = new SwerveTelemetry(oi.drivetrain.frontRight);
        SwerveTelemetry backRightTelemetry = new SwerveTelemetry(oi.drivetrain.backRight);
        //SendableRegistry.add(frontLeftTelemetry, "Swerve");

        
       
        SendableRegistry.addLW(frontLeftTelemetry, "FL Swerve");
        SendableRegistry.addLW(backLeftTelemetry, "BL Swerve");
        SendableRegistry.addLW(frontRightTelemetry, "FR Swerve");
        SendableRegistry.addLW(backRightTelemetry, "BR Swerve");
    }
}