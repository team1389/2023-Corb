package frc.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Limelight;

public class AutoAlign extends CommandBase{

    public Limelight april;
    private final Drivetrain drivetrain;

     public AutoAlign(int aprilTagPipeline, Drivetrain drivetrain) {
        april = new Limelight(aprilTagPipeline);
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double speed = 0.4;

        // Use formula to find angle robot should drive at
        double targetAngle = 0;
        double rotAngle = april.X();

        rotAngle = april.X();

        SmartDashboard.putNumber("Rotation Angle", Math.toDegrees(rotAngle));
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));

        if (Math.abs(Math.toDegrees(rotAngle)) < 12) {
            drivetrain.stopModules();
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,
                    0, rotAngle*speed);
            SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    
}
package frc.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.subsystems.Drivetrain;

public class AutoAlign extends CommandBase{
    private final Drivetrain drivetrain;

    public AutoAlign(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){

        SmartDashboard.putNumber("LimelightX", LimelightHelpers.getTX(""));
        SmartDashboard.putString("InsideAutoAlign", "success");
        SmartDashboard.putNumber("LimelightY", LimelightHelpers.getTY(""));
        SmartDashboard.putNumber("LimelightArea", LimelightHelpers.getTA(""));
    }
}
