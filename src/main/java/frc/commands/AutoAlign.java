package frc.commands;
import frc.subsystems.Limelight;

public class AutoAlign {

    public Limelight april;

     public AutoAlign(int aprilTagPipeline) {
        april = new Limelight(aprilTagPipeline);
    }

    public double getAlignment(double xPos) {
        if (april.valid()) {
            return (april.X()-xPos)/35;
        }
        return 0;
    }

    public boolean closeEnough() {
        return (april.area() > 1.3);
    }
}
