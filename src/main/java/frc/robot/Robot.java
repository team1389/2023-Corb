package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
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
        CameraServer.startAutomaticCapture();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     * 
     * Anthony Noya wuz here . 
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        double voltage = pdh.getVoltage();
        SmartDashboard.putNumber("Voltage", voltage);

        SmartDashboard.putNumber("wrsit 2 Current", pdh.getCurrent(2));
        SmartDashboard.putNumber("wrist 3 Current", pdh.getCurrent(3));
        SmartDashboard.putNumber("wrist 4 Current", pdh.getCurrent(4));
        


        // SmartDashboard.putNumber("BR Drive Current", pdh.getCurrent(1));
        // SmartDashboard.putNumber("BL Drive Current", pdh.getCurrent(9));
        // SmartDashboard.putNumber("FR Turn Current", pdh.getCurrent(18));
        // SmartDashboard.putNumber("FL Turn Current", pdh.getCurrent(11));
        // SmartDashboard.putNumber("BL Turn Current", pdh.getCurrent(8));
        // SmartDashboard.putNumber("BR Turn Current", pdh.getCurrent(0));
        // SmartDashboard.putNumber("Total Current", pdh.getTotalCurrent());

    }

    @Override
    public void autonomousInit() {
        autoCommand = oi.getAutoCommand();

        // schedule the autonomous command (example)
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }


    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (autoCommand != null) {
    //     autoCommand.cancel();
    //   }
    
    }

    @Override
    public void teleopInit() {
        // SwerveTelemetry frontLeftTelemetry = new SwerveTelemetry(oi.drivetrain.frontLeft);
        // SwerveTelemetry backLeftTelemetry = new SwerveTelemetry(oi.drivetrain.backLeft);
        // SwerveTelemetry frontRightTelemetry = new SwerveTelemetry(oi.drivetrain.frontRight);
        SwerveTelemetry backRightTelemetry = new SwerveTelemetry(oi.drivetrain.backRight);
        //SendableRegistry.add(frontLeftTelemetry, "Swerve");

        //SmartDashboard.putNumber("FL angular", frontLeftTelemetry.get);
       
        // SendableRegistry.addLW(frontLeftTelemetry, "FL Swerve");
        // SendableRegistry.addLW(backLeftTelemetry, "BL Swerve");
        // SendableRegistry.addLW(frontRightTelemetry, "FR Swerve");
        // SendableRegistry.addLW(backRightTelemetry, "BR Swerve");


        oi.drivetrain.frontLeft.resetEncoders();
        oi.drivetrain.backLeft.resetEncoders();
        oi.drivetrain.frontRight.resetEncoders();
        oi.drivetrain.backRight.resetEncoders();
        oi.drivetrain.setAngleAdjustment(180);

        oi.arm.controllerInterrupt = true;
    }

        

    @Override
    public void teleopPeriodic() {
        // SmartDashboard.putNumber("FL angle", Math.toDegrees(frontLeftTelemetry.getAngle()));
        SmartDashboard.putNumber("FL speed", frontLeftTelemetry.getSpeed());


        // SmartDashboard.putNumber("BL angle", Math.toDegrees(backLeftTelemetry.getAngle()));
        SmartDashboard.putNumber("BL speed", backLeftTelemetry.getSpeed());

        // SmartDashboard.putNumber("FR angle", Math.toDegrees(frontRightTelemetry.getAngle()));
        SmartDashboard.putNumber("FR speed", frontRightTelemetry.getSpeed());

        // SmartDashboard.putNumber("BR angle", Math.toDegrees(backRightTelemetry.getAngle()));
        SmartDashboard.putNumber("BR speed", backRightTelemetry.getSpeed());

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testInit() {
        // SwerveTelemetry frontLeftTelemetry = new SwerveTelemetry(oi.drivetrain.frontLeft);
        // SwerveTelemetry backLeftTelemetry = new SwerveTelemetry(oi.drivetrain.backLeft);
        // SwerveTelemetry frontRightTelemetry = new SwerveTelemetry(oi.drivetrain.frontRight);
        // SwerveTelemetry backRightTelemetry = new SwerveTelemetry(oi.drivetrain.backRight);
        // //SendableRegistry.add(frontLeftTelemetry, "Swerve");

        
       
        // SendableRegistry.addLW(frontLeftTelemetry, "FL Swerve");
        // SendableRegistry.addLW(backLeftTelemetry, "BL Swerve");
        // SendableRegistry.addLW(frontRightTelemetry, "FR Swerve");
        // SendableRegistry.addLW(backRightTelemetry, "BR Swerve");
    }
}