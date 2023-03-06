package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.*;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LEDPanel extends SubsystemBase {

    private final CANdle m_candle = new CANdle(CANdle);

    private final static int kLED_COLUMNS = 32;
    private final static int kLED_ROWS = 8;

    private boolean m_isPanelDisabled = false;

  /** Creates a new LedPannel. */
  public LEDPanel() {
        CANdleConfiguration configALL = new CANdleConfiguration();
        configALL.disableWhenLOS = false;
        configALL.stripType = LEDStripType.GRB;
        m_candle.configAllSettings(configALL);
  }

  public void setLED(int r, int g, int b, int index) {
    m_candle.setLEDs(r, g, b, 255, index, 1);
  }

    @Override
  public void periodic() {
    //Only run if disabled
    if (!DriverStation.isDisabled()){
        //Turn all lights off
        m_candle.setLEDs(0, 0, 0);
        m_isPanelDisabled = true;
    }
  }
}