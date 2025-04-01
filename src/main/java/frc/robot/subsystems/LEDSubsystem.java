// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 0;
  private static final int kLength = 320;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  
  private int direction = 2;
  private static final int GOLD_WIDTH = 10;
  private int speed = 1; // Adjustable speed
  private Timer timer = new Timer();
  private double delay = 0.005; // Delay between updates
  private int bounceStart = 110;
  private int bounceEnd = 176;
  private int goldPosition = bounceStart;


  //Setup Rainbow
  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  // Our LED strip has a density of 120 LEDs per meter -- does it?
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);


// Create an LED pattern that displays the first half of a strip as solid red,
// and the second half of the strip as solid blue.
LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kDarkGreen, 0.5, Color.kDarkGreen));
LEDPattern stepsDark = steps.atBrightness(Percent.of(50));

LEDPattern breathe = steps.breathe(Seconds.of(10));

  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    timer.restart();

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kGreen)));
  }

  public void setLEDs(int r, int g, int b) {
    for (int i = 0; i < m_buffer.getLength(); i++) {
        m_buffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_buffer);
  }

  public void turnOffLeds()
  {
    setLEDs(0, 0, 0);
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to
    // display

    // // Update the buffer with the rainbow animation
    // m_scrollingRainbow.applyTo(m_buffer);

    // Apply the LED pattern to the data buffer
    // Not certain if this works one time for animated patterns or needs update every cycle
    // If every cycle, probably need a generic LEDPattern that methods / commands assign based
    // on desired effect.
    // stepsDark.applyTo(m_buffer);

    // Write to the actual LEDs
    // m_led.setData(m_buffer);

    update();

  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }

  private void setDarkGreen() {
    for (int i = 0; i < kLength; i++) {
        m_buffer.setRGB(i, 0, 50, 0); // Dark Green
    }
}

private void setGoldPatch() {
    for (int i = 0; i < GOLD_WIDTH; i++) {
        int index = goldPosition + i;
        if (index < kLength) {
            // m_buffer.setRGB(index, 255, 215, 0); // Gold
            m_buffer.setRGB(index, 150, 0, 0); // Gold
        }
    }
}

public void update() {
  if (timer.advanceIfElapsed(delay)) {
    setDarkGreen(); // Reset all to Dark Green
    setGoldPatch(); // Apply Gold Patch

    goldPosition += direction * speed;
    if (goldPosition + GOLD_WIDTH >= bounceEnd || goldPosition <= bounceStart) {
      direction *= -1; // Reverse direction at edges
    }

    m_led.setData(m_buffer);
  }
}

public void setSpeed(int newSpeed) {
  speed = Math.max(1, newSpeed); // Ensure speed is at least 1
}
}
