package frc.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
    private final Drivetrain drivetrain;

    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        double pitch = Math.toRadians(drivetrain.getPitch());
        double roll = Math.toRadians(drivetrain.getRoll());
        double speed = 0.4;

        // Use formula to find angle robot should drive at
        double targetAngle = Math.asin(Math.sin(roll)/(Math.sqrt((Math.pow(Math.sin(pitch),2))+(Math.pow(Math.sin(roll),2)))));
        double slopeAngle = Math.asin((Math.sqrt((Math.pow(Math.sin(pitch),2))+(Math.pow(Math.sin(roll),2)))));

        targetAngle = (pitch < 0) ? Math.PI-targetAngle : targetAngle;

        SmartDashboard.putNumber("Angle Angle", Math.toDegrees(slopeAngle));
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Pitch", drivetrain.getPitch());
        SmartDashboard.putNumber("Roll", drivetrain.getRoll());
        

        if(Math.abs(Math.toDegrees(slopeAngle)) < 12){
            
            drivetrain.stopModules();
            
            
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed*Math.cos(targetAngle), speed*Math.sin(targetAngle), 0); 
            SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
        
    }
    

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}
