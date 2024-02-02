package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimeLightVision extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx1 = table.getEntry("tx");
    NetworkTableEntry ty1 = table.getEntry("ty");
    NetworkTableEntry ta1 = table.getEntry("ta");
    
    double tx = LimelightHelpers.getTX("");
    double ty = LimelightHelpers.getTY("");
    double ta = LimelightHelpers.getTA(""); //area


    public LimeLightVision(){
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        LimelightHelpers.setCropWindow("",-1,1,-1,1);
        LimelightHelpers.getTX("");
        
    }
    public void updatePose(){
            // pid loop to return yaw (and then turn the robot) (from pigeon)
            // use x,y,area and pid loop to move robot from current point to set point
            // move angle and shoot
            


        }

    // poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getModulePositions());
    // updateFieldPose();
    
    @Override
    public void periodic() {
        //read values periodically
        double x = tx1.getDouble(0.0);
        double y = ty1.getDouble(0.0);
        double area = ta1.getDouble(0.0);

        // //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX1", x);
        SmartDashboard.putNumber("LimelightY2", y);
        SmartDashboard.putNumber("LimelightArea3", area);
        
        SmartDashboard.putNumber("LimelightX", LimelightHelpers.getTX(""));
        SmartDashboard.putString("InsideAutoAlign", "success");
        SmartDashboard.putNumber("LimelightY", LimelightHelpers.getTY(""));
        SmartDashboard.putNumber("LimelightArea", LimelightHelpers.getTA(""));
    }

    
}
