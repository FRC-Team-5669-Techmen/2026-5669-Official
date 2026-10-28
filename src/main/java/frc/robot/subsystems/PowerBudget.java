package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Power;
import frc.robot.commands.VacuumCommand;

/**
 * POWER BUDGET (Oct 27 night): keeps the battery out of brownout while everything runs at once.
 *
 * <p>Vacuum and dump mode run 8 drive/steer motors, the 2 flywheel motors, the spindexer, the
 * uptake and the roller together: a worst moment (hard acceleration + a flywheel recovering) can
 * ask for 300+ A, and a match battery (~0.02 ohm plus wiring) sags under 6 V there = brownout,
 * every motor goes limp. Top speed costs little; ACCELERATING and PUSHING are what draw current,
 * so this caps the drive motors' SUPPLY current (not the speed). 6328 does a full version of this
 * (battery and breaker model, 5-35 A per module in teleop).
 * <ul>
 *   <li>NORMAL: 40 A supply per drive motor (as configured).</li>
 *   <li>HEAVY (vacuum or dump mode on): {@link Power#kHeavyDriveSupplyAmps} per drive motor. Same top
 *       speed, a bit softer acceleration and pushing.</li>
 *   <li>SAG (heavy, and the battery still dropped under {@link Power#kSagVolts}): lower still, until it
 *       is back over {@link Power#kRecoverVolts} for a second.</li>
 *   <li>OVERRIDE: Driver LEFT STICK CLICK held = full drive power right now (to escape, push, or if
 *       something acts up). Dashboard switch "Power Budget ON" (saved) turns the whole thing off.</li>
 * </ul>
 * The stator (slip) limit is never touched. Dashboard "Power Budget".
 */
public class PowerBudget extends SubsystemBase {
    private static final String kOnKey = "Power Budget ON";
    private final List<TalonFX> drives = new ArrayList<>();
    private final List<CurrentLimitsConfigs> configs = new ArrayList<>();
    private final BooleanSupplier driverOverride;
    private double applied = Double.NaN;
    private double lastSend = 0, lowSince = -1, highSince = -1;
    private boolean sag = false;

    private final AimSubsystem aim;

    public PowerBudget(CommandSwerveDrivetrain drivetrain, AimSubsystem aim, BooleanSupplier driverOverride) {
        this.aim = aim;
        this.driverOverride = driverOverride;
        for (var m : drivetrain.getModules()) {
            TalonFX d = m.getDriveMotor();
            CurrentLimitsConfigs c = new CurrentLimitsConfigs();
            d.getConfigurator().refresh(c); // once at boot (blocking is fine here)
            drives.add(d);
            configs.add(c);
        }
        edu.wpi.first.wpilibj.Preferences.initBoolean(kOnKey, true);
        SmartDashboard.putBoolean(kOnKey, edu.wpi.first.wpilibj.Preferences.getBoolean(kOnKey, true));
    }

    @Override
    public void periodic() {
        boolean on = SmartDashboard.getBoolean(kOnKey, true);
        if (on != edu.wpi.first.wpilibj.Preferences.getBoolean(kOnKey, true))
            edu.wpi.first.wpilibj.Preferences.setBoolean(kOnKey, on);
        boolean heavy = on && DriverStation.isTeleopEnabled() && (VacuumCommand.dumping() || aim.vacuum());
        boolean override = driverOverride.getAsBoolean();
        double now = Timer.getFPGATimestamp();
        double volts = RobotController.getBatteryVoltage();

        // Sag: under kSagVolts for 0.1 s = step down; back over kRecoverVolts for 1 s = step up.
        if (heavy && volts < Power.kSagVolts) { if (lowSince < 0) lowSince = now; } else lowSince = -1;
        if (volts > Power.kRecoverVolts) { if (highSince < 0) highSince = now; } else highSince = -1;
        if (lowSince >= 0 && now - lowSince > 0.1) sag = true;
        if (!heavy || (highSince >= 0 && now - highSince > 1.0)) sag = false;

        double amps = Power.kNormalDriveSupplyAmps;
        String text = on ? "NORMAL" : "OFF (switch)";
        if (heavy && override) text = "OVERRIDE: full drive power (Driver left stick click)";
        else if (heavy && sag) { amps = Power.kSagDriveSupplyAmps; text = String.format("SAG %.1f V: drive %.0f A", volts, amps); }
        else if (heavy) { amps = Power.kHeavyDriveSupplyAmps; text = String.format("HEAVY: drive %.0f A per motor", amps); }

        // Send on a change, and again once a second while limited (sent without waiting for a reply,
        // so the loop never stalls; the repeat covers a dropped frame).
        if (amps != applied || (amps != Power.kNormalDriveSupplyAmps && now - lastSend > 1.0)) {
            for (int i = 0; i < drives.size(); i++) {
                CurrentLimitsConfigs c = configs.get(i);
                c.SupplyCurrentLimit = amps;
                c.SupplyCurrentLimitEnable = true;
                drives.get(i).getConfigurator().apply(c, 0.0);
            }
            if (amps != applied) System.out.println("POWER | drive supply " + amps + " A per motor (" + text + ")");
            applied = amps;
            lastSend = now;
        }
        SmartDashboard.putString("Power Budget", text);
        DogLog.log("Power/DriveSupplyAmps", amps);
        DogLog.log("Power/Sag", sag);
    }
}
