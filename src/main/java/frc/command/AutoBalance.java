package frc.command;

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
        double maxSpeed = 0.33;
        double tempSpeed;

        // Use formula to find angle robot should drive at
        double targetAngle = Math.asin(Math.sin(roll)/(Math.sqrt((Math.pow(Math.sin(pitch),2))+(Math.pow(Math.sin(roll),2)))));
        double slopeAngle = Math.asin((Math.sqrt((Math.pow(Math.sin(pitch),2))+(Math.pow(Math.sin(roll),2)))));

        targetAngle = (pitch < 0) ? Math.PI-targetAngle : targetAngle;

        SmartDashboard.putNumber("Angle Angle", Math.toDegrees(slopeAngle));
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Pitch", drivetrain.getPitch());
        SmartDashboard.putNumber("Roll", drivetrain.getRoll());
        
        //if(Math.abs(Math.toDegrees(pitch))<1 && Math.abs(Math.toDegrees(roll))<1){
          //  drivetrain.stopModules();
        //}
        if(15>Math.abs(Math.toDegrees(slopeAngle))){
           drivetrain.stopModules();
        }
        else{
            tempSpeed = Math.min((Math.toDegrees(slopeAngle)/15)*maxSpeed, maxSpeed);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(tempSpeed*Math.cos(targetAngle), tempSpeed*Math.sin(targetAngle), 0); 
            SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}
