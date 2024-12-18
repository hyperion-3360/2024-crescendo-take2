// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public void setRainbowColor() {

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
  }

  public Command rainbow() {

    return this.runOnce(() -> setRainbowColor());
  }

  // blue when climber
  public void setBlueColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 255);
    }
    ledStrip.setData(ledBuffer);
  }

  public Command blue() {

    return this.runOnce(() -> setBlueColor());
  }

  // green when a note is in the elevator\
  public void setGreenColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    ledStrip.setData(ledBuffer);
  }

  public Command green() {

    return this.runOnce(() -> setGreenColor());
  }

  // red when helevator is high
  public void setRedColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    ledStrip.setData(ledBuffer);
  }

  public Command red() {

    return this.runOnce(() -> setRedColor());
  }

  // teal when intaking
  public void setLavenderColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 30, 20, 90);
    }
    ledStrip.setData(ledBuffer);
  }

  public Command lavender() {

    return this.runOnce(() -> setLavenderColor());
  }

  // yellow when...
  // could be removed ( no subsystem tu use)
  public void setYellowColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 230, 90, 0);
    }
    ledStrip.setData(ledBuffer);
  }

  public Command yellow() {

    return this.runOnce(() -> setYellowColor());
  }

  // orange when elevator shoots
  public void setOrangeColor() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 230, 20, 0);
    }
    ledStrip.setData(ledBuffer);
  }

  public Command orange() {

    return this.runOnce(() -> setOrangeColor());
  }

  public void setColorFinder() {
    int RED = (int) SmartDashboard.getNumber("colorFinderRed", 0);
    int GREEN = (int) SmartDashboard.getNumber("colorFinderGreen", 0);
    int BLUE = (int) SmartDashboard.getNumber("colorFinderBlue", 0);
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, RED, GREEN, BLUE);
    }
    ledStrip.setData(ledBuffer);
  }

  public Command colorFinder() {

    return this.runOnce(() -> setColorFinder());
  }

  @Override
  public void periodic() {
    // setColorFinder();
    if (!DriverStation.isAutonomous()) {
      rainbow().cancel();
    }
  }
}
