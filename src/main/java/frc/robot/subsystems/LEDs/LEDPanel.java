package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.*;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDPanel extends SubsystemBase {

    private final CANdle m_candle = new CANdle(CANdle);

    private final static int kLED_COLUMNS = 33;
    private final static int kLED_ROWS = 8;
    private final static int kSTRIP_START = kLED_COLUMNS * kLED_ROWS;
    private final static int kSTRIP_LENGTH = 48;
    private final static int kLED_TOTAL = kLED_COLUMNS * kLED_ROWS + kSTRIP_LENGTH;

    private boolean m_isPanelDisabled = false;

    /** Creates a new LedPannel. */
    public LEDPanel() {
        CANdleConfiguration configALL = new CANdleConfiguration();
        configALL.disableWhenLOS = false;
        configALL.stripType = LEDStripType.GRB;
        configALL.brightnessScalar = 0.1; // dim the LEDs to half brightness
        // configALL.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configALL, 100);
        m_Timer.start();
    }

    private static Timer m_Timer = new Timer();

    @Override
    public void periodic() {
        // limit to 10x a second
        if (m_Timer.advanceIfElapsed(0.1)) {
            // Only run if not disabled
            if (DriverStation.isDisabled()) {
                // Turn all lights red
                // m_candle.setLEDs(255, 0, 0);
            }

            if (RobotContainer.m_Tracking.isTrackingTarget()) {
                this.showTrackingStatusGreen();
            } else {
                this.showTrackingStatusRed();
            }

            if (RobotContainer.m_GridSelector.isCube()) {
                this.showCube();
            } else {
                this.showCone();
            }
        }

    }

    public void showCone() {
        m_candle.setLEDs(255, 255, 0, 0, 0, 33 * 8);
    }

    public void showCube() {
        m_candle.setLEDs(138, 43, 226, 0, 0, 33 * 8);
    }

    public void showVisionStatusGreen() {
        m_candle.setLEDs(0, 255, 0, 0, kSTRIP_START, kSTRIP_LENGTH);
    }

    public void showVisionStatusRed() {
        m_candle.setLEDs(255, 0, 0, 0, kSTRIP_START, kSTRIP_LENGTH);
    }

    public void showVisionStatusYellow() {
        m_candle.setLEDs(255, 255, 0, 0, kSTRIP_START, kSTRIP_LENGTH);
    }

    public void showTrackingStatusGreen() {
        m_candle.setLEDs(0, 255, 0, 0, 100, 80);
    }

    public void showTrackingStatusRed() {
        m_candle.setLEDs(255, 0, 0, 0, 100, 80);
    }

    public void showTrackingStatusYellow() {
        m_candle.setLEDs(255, 255, 0, 0, 100, 80);
    }

}