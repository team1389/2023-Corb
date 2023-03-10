package frc.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;
import frc.util.SizeLimitedQueue;

public class AutoBalance extends CommandBase {
    private final Drivetrain drivetrain;
    private final SizeLimitedQueue queue;
    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        queue = new SizeLimitedQueue(100);

        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        double pitch = Math.toRadians(drivetrain.getPitch());
        double roll = Math.toRadians(drivetrain.getRoll());
        double maxSpeed = 0.4;
        double tempSpeed;

        // Use formula to find angle robot should drive at
        double targetAngle = Math.asin(Math.sin(roll)/(Math.sqrt((Math.pow(Math.sin(pitch),2))+(Math.pow(Math.sin(roll),2)))));
        double slopeAngle = Math.asin((Math.sqrt((Math.pow(Math.sin(pitch),2))+(Math.pow(Math.sin(roll),2)))));

        targetAngle = (pitch < 0) ? Math.PI-targetAngle : targetAngle;

        SmartDashboard.putNumber("Angle Angle", Math.toDegrees(slopeAngle));
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Pitch", drivetrain.getPitch());
        SmartDashboard.putNumber("Roll", drivetrain.getRoll());
        

        if(16>Math.abs(Math.toDegrees(slopeAngle))){
            queue.add(slopeAngle);

            if(2.5>Math.abs(Math.toDegrees(slopeAngle))){
                drivetrain.stopModules();
            }
            else if(10>Math.abs(Math.toDegrees(slopeAngle))){

                if(queue.length() >= 20) {
                    maxSpeed += 0.0025*queue.getSum();
                }
            }
        }
        if(2.5<Math.abs(Math.toDegrees(slopeAngle))){
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
