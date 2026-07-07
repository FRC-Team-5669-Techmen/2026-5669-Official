package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
  // Hardware references are kept private and accessed only through the safe methods below
  private DoubleSolenoid m_cylinder;
  private Solenoid m_other;

  /**
   * Constructor for a Single Solenoid setup (e.g., the climb piston)
   * @param pcmId The CAN ID of the PCM.
   * @param channel The channel on the PCM.
   */
  public PneumaticSubsystem(int pcmId, int channel) {
    m_other = new Solenoid(pcmId, PneumaticsModuleType.CTREPCM, channel);
    m_other.set(false); // Default state
  }

  /**
   * Constructor for a Double Solenoid setup (e.g., the intake pistons)
   * @param pcmId The CAN ID of the PCM.
   * @param forwardChannel The forward channel on the PCM.
   * @param reverseChannel The reverse channel on the PCM.
   */
  public PneumaticSubsystem(int pcmId, int forwardChannel, int reverseChannel) {
    m_cylinder = new DoubleSolenoid(
        pcmId, 
        PneumaticsModuleType.CTREPCM, 
        forwardChannel, 
        reverseChannel
    );
    m_cylinder.set(Value.kReverse); // Default state
  }

  /**
   * Safely extends whichever pneumatic cylinder was initialized.
   */
  public void extend() {
    if (m_cylinder != null) {
      m_cylinder.set(Value.kForward);
    } else if (m_other != null) {
      m_other.set(true);
    }
  }

  /**
   * Safely retracts whichever pneumatic cylinder was initialized.
   */
  public void retract() {
    if (m_cylinder != null) {
      m_cylinder.set(Value.kReverse);
    } else if (m_other != null) {
      m_other.set(false);
    }
  }

  /**
   * Smart toggle: Automatically detects and toggles whichever solenoid type is active.
   */
  public void toggle() {
    if (m_cylinder != null) {
      m_cylinder.toggle();
    } else if (m_other != null) {
      m_other.toggle();
    }
  }
}