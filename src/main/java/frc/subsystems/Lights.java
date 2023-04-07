package frc.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.LightConstants;

public class Lights extends SubsystemBase {
    AddressableLED ledStrip;
    AddressableLEDBuffer buffer;
    public int hueee = 0;
    public boolean isRainbowing = true;

    public Lights() {
        ledStrip = new AddressableLED(LightConstants.LED_PORT);
        buffer = new AddressableLEDBuffer(LightConstants.COUNT);

        ledStrip.setLength(buffer.getLength());

        ledStrip.setData(buffer);
        ledStrip.start();
        
    }

    //id == 0 for left, 1 for right, 2 for both
    public void setColor(int red, int green, int blue) {
        isRainbowing = false;
          for (var i = 0; i < buffer.getLength(); i++) {
              // Sets the specified LED to the RGB values for red
              buffer.setRGB(i, red, green, blue);
          }
        
          ledStrip.setData(buffer);
        
        
    }

    
    public void rainbow() {
        var firstPixelHue = hueee%180;

        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (firstPixelHue + (i * 180 / buffer.getLength())) % 180;
          // Set the value
          buffer.setHSV(i, hue, 255, 128);
          buffer.setHSV(i, hue, 255, 128);
        }
      }

      @Override
      public void periodic() {
        if(isRainbowing) {
          rainbow();
        }
        // Set the LEDs
        ledStrip.setData(buffer);
      }

    
}
