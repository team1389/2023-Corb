package frc.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.LightConstants;

public class Lights extends SubsystemBase {
    AddressableLED leftLedStrip;
    AddressableLED rightLedStrip;
    AddressableLEDBuffer leftBuffer, rightBuffer;

    public Lights() {
        leftLedStrip = new AddressableLED(LightConstants.LEFT_LED_PORT);
        rightLedStrip = new AddressableLED(LightConstants.RIGHT_LED_PORT);

        leftBuffer = new AddressableLEDBuffer(LightConstants.leftCount);
        rightBuffer = new AddressableLEDBuffer(LightConstants.rightCount);

        leftLedStrip.setLength(leftBuffer.getLength());
        rightLedStrip.setLength(rightBuffer.getLength());

        leftLedStrip.setData(leftBuffer);
        leftLedStrip.start();
        rightLedStrip.setData(leftBuffer);
        rightLedStrip.start();
    }

    //id == 0 for left, 1 for right, 2 for both
    public void setColor(int id, int red, int green, int blue) {
        if(id == 0) {
          for (var i = 0; i < leftBuffer.getLength(); i++) {
              // Sets the specified LED to the RGB values for red
              leftBuffer.setRGB(i, red, green, blue);
          }
        
          leftLedStrip.setData(leftBuffer);
        }
        else if (id == 1) {
          for (var i = 0; i < rightBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            rightBuffer.setRGB(i, red, green, blue);
          }
      
          rightLedStrip.setData(rightBuffer);
        }
        else {
          for (var i = 0; i < leftBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            leftBuffer.setRGB(i, red, green, blue);
          }
          for (var i = 0; i < rightBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            rightBuffer.setRGB(i, red, green, blue);
          }

          leftLedStrip.setData(leftBuffer);
          rightLedStrip.setData(rightBuffer);
        }
        
    }

    public void rainbow() {
        var firstPixelHue = 0;

        // For every pixel
        for (var i = 0; i < leftBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (firstPixelHue + (i * 180 / leftBuffer.getLength())) % 180;
          // Set the value
          leftBuffer.setHSV(i, hue, 255, 128);
          rightBuffer.setHSV(i, hue, 255, 128);
        }
        
        // Increase by to make the rainbow "move"
        firstPixelHue += 3;
        // Check bounds
        firstPixelHue %= 180;
      }

      @Override
      public void periodic() {
        // Set the LEDs
        leftLedStrip.setData(leftBuffer);
        rightLedStrip.setData(rightBuffer);
      }

    
}
