package frc.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.subsystems.SwerveModule;

public class SwerveTelemetry implements Sendable {
    SwerveModule swerveWheel;

    public SwerveTelemetry(SwerveModule wheel) {
        swerveWheel = wheel;
        SendableRegistry.addLW(this, "joe");


    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("SwerveTelemetry");
      builder.addDoubleProperty("Speed", () -> getSpeed(), null);
      builder.addDoubleProperty("Angle", () -> getAngle(), null);
      builder.addDoubleProperty("Target Angle", () -> getTargetAngle(), null);
      builder.addDoubleProperty("Target Speed", () -> getTargetSpeed(), null);
      builder.addDoubleProperty("Absolute Angle", () -> getAbsAngle(), null);
      builder.addBooleanProperty("Inverted", () -> getInverted(), null);
    }


    public double getSpeed() {
        return swerveWheel.getState().speedMetersPerSecond;
    }
    public double getAngle() {
        return swerveWheel.getState().angle.getDegrees();
    }
    public double getTargetAngle() {
        return swerveWheel.targetAngle;
    }
    public double getTargetSpeed() {
        return swerveWheel.targetSpeed;
    }
    public double getAbsAngle() {
        return Math.toDegrees(swerveWheel.getAbsoluteEncoderRad());
    }

    public boolean getInverted() {
        //return swerveWheel.isInverted();
        return false;
    }

  }