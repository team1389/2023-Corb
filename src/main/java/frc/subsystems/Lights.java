package frc.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.LightConstants;

public class Lights extends SubsystemBase {
    AddressableLED ledStrip;
    AddressableLEDBuffer ledBuffer;

    public Lights() {
        ledStrip = new AddressableLED(LightConstants.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(LightConstants.ledCount);

        ledStrip.setLength(ledBuffer.getLength());

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public void setColor(int red, int green, int blue) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, red, green, blue);
         }
         
         ledStrip.setData(ledBuffer);
    }

    private void rainbow() {
        var firstPixelHue = 0;

        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (firstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        firstPixelHue += 3;
        // Check bounds
        firstPixelHue %= 180;
      }

      @Override
      public void periodic() {
        // Fill the buffer with a rainbow
        rainbow();
        // Set the LEDs
        ledStrip.setData(ledBuffer);
      }

    
}
