package frc.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;

public class AutoBalanceController extends CommandBase {
    private final Drivetrain drivetrain;
    private final int targetAngle = 0;
    double kp = 0.01625, ki = 0, kd = 0;
    PIDController pidController = new PIDController(kp, ki, kd);

    public AutoBalanceController(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double pitch = Math.toRadians(drivetrain.getPitch());
        double roll = Math.toRadians(drivetrain.getRoll());
        double driveAngle = Math
                .asin(Math.sin(roll) / (Math.sqrt((Math.pow(Math.sin(pitch), 2)) + (Math.pow(Math.sin(roll), 2)))));
        double tempSpeed;
        // Read the current angle from a sensor
        double currentAngle = getCurrentAngle(pitch, roll);

        // very very very important code. DO NOT DELETE OR IT WILL DRIVE ONE DIRECTION
        driveAngle = (pitch < 0) ? Math.PI - targetAngle : targetAngle;

        SmartDashboard.putNumber("Angle Angle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("Drive Angle", Math.toDegrees(driveAngle));
        SmartDashboard.putNumber("Pitch", drivetrain.getPitch());
        SmartDashboard.putNumber("Roll", drivetrain.getRoll());

        if (2.5 > Math.abs(Math.toDegrees(currentAngle))) {
            drivetrain.stopModules();
        } else {

            // Calculate the control output using a PID controller
            tempSpeed = Math.abs(MathUtil.clamp(calculatePIDControlOutput(currentAngle), -0.33, 0.33));
            SmartDashboard.putNumber("Speed i am Speed", tempSpeed);

            // leave this stuff alone
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(tempSpeed * Math.cos(driveAngle),
                    tempSpeed * Math.sin(driveAngle), 0);
            SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    private double getCurrentAngle(double pitch, double roll) {
        return Math.asin((Math.sqrt((Math.pow(Math.sin(pitch), 2)) + (Math.pow(Math.sin(roll), 2)))));
    }

    private double calculatePIDControlOutput(double currentAngle) {
        return pidController.calculate(Math.toDegrees(currentAngle), Math.toDegrees(targetAngle));
    }
}
