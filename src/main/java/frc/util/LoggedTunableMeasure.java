// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.mutable.GenericMutableMeasureImpl;
import frc.robot.constants.RobotConstants;
/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableMeasure<U extends Unit> implements Supplier<Measure<U>> {
    private static final String tableKey = "TunableNumbers";

    private final String key;
    private final GenericMutableMeasureImpl<U> dashboardMeasure;
    private final LoggedDashboardNumber dashboardNumber;
    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableMeasure(String dashboardKey, Measure<U> defaultValue) {
        this(dashboardKey, defaultValue, defaultValue.unit());
    }
    public LoggedTunableMeasure(String dashboardKey, Measure<U> defaultValue, U dashboardUnit) {
        this.key = tableKey + "/" + dashboardKey;
        this.dashboardMeasure = new GenericMutableMeasureImpl<U>(defaultValue.in(dashboardUnit), defaultValue.baseUnitMagnitude(), dashboardUnit);
        if(RobotConstants.tuningMode) {
            dashboardNumber = new LoggedDashboardNumber(key, dashboardMeasure.magnitude());
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public Measure<U> get() {
        if(RobotConstants.tuningMode) {
            dashboardMeasure.mut_setMagnitude(dashboardNumber.get());
        }
        return dashboardMeasure;
    }

    public double in(U unit) {
        return get().in(unit);
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged(int id) {
        double currentValue = dashboardNumber.get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static boolean hasChanged(int id, LoggedTunableMeasure<?>... tunableMeasures) {
        return Arrays.stream(tunableMeasures).anyMatch((tunableMeasure) -> tunableMeasure.hasChanged(id));
    }
}
