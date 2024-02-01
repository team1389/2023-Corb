package frc.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.subsystems.Drivetrain;

public class AutoAlign extends CommandBase{

    private final Drivetrain drivetrain;
    double tx;

     public AutoAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }


    @Override
    public void execute() {

        

        tx = LimelightHelpers.getTX("");

        double speed = 0.1;

        //math
        double targetAngle = 0;
        double rotAngle = tx;

        rotAngle = tx;

        SmartDashboard.putNumber("Rotation Angle", Math.toDegrees(rotAngle));
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));

        if (Math.abs(rotAngle) < 0.5) {
            drivetrain.stopModules();
            //rotAngle*speed
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,
                    0, speed * rotAngle);
            SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    
}
