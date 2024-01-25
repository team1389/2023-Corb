package frc.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private int pipeline = 1;
	private double invalidArea = 0.4;

	public Limelight(int Pipline) {
		if (Pipline >= 0 && Pipline < 10) {
			pipeline = Pipline;
		}
	}

	public void setPip() {
		table.getEntry("pipeline").setNumber(pipeline);
	}

	public double X() {
		setPip();
		return table.getEntry("tx").getDouble(0);
	}

    public double Y() {
		setPip();
		return table.getEntry("ty").getDouble(0);
	}

    public double area() {
		setPip();
		return table.getEntry("ta").getDouble(0);
	}

	public double width() {
		setPip();
		return table.getEntry("thor").getDouble(0);
	}

	public double height() {
		setPip();
		return table.getEntry("tvert").getDouble(0);
	}

	public boolean valid() {
		setPip();
		if (table.getEntry("ta").getDouble(0) > invalidArea) {
			return true;
		} else {
			return false;
		}
	}

	public boolean inRange(double value, double setting, double errorRange) {
		return ((value >= setting - errorRange) && (value <= setting + errorRange));
	}
}