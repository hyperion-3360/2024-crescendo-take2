// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;

  public LEDs() {
    ledStrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(60);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  // rainbow when game ends
  public Command rainbow() {

    /** (Rainbow first pixel hue) */
    int rainbowFirstPixelHue = 0;

    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;

      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    return this.runOnce(() -> ledStrip.setData(ledBuffer));
  }

  // blue when climber
  public void setBlueColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 255);
    }
  }

  public Command blue() {

    return this.runOnce(() -> setBlueColor());
  }

  // green when a note is in the elevator\
  public void setGreenColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public Command green() {

    return this.runOnce(() -> setGreenColor());
  }

  // red when helevator is high
  public void setRedColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
  }

  public Command red() {

    return this.runOnce(() -> setRedColor());
  }

  // teal when intaking
  public void setTealColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 128, 128);
    }
  }

  public Command teal() {

    return this.runOnce(() -> setTealColor());
  }

  // yellow when...
  // could be removed (more green than yellow and no subsystem tu use)
  public void setYellowColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 255, 0);
    }
  }

  public Command yellow() {

    return this.runOnce(() -> setYellowColor());
  }

  // orange when elevator shoots
  // to make orange (it's kind of yellow)
  public void setOrangeColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 255, 0);
    }
  }

  public Command orange() {

    return this.runOnce(() -> setOrangeColor());
  }

  @Override
  public void periodic() {

    ledStrip.setData(ledBuffer);
  }
}
