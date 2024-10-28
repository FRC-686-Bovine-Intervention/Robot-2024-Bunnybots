package frc.robot.util;

import java.util.LinkedHashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Alert.AlertType;

public enum Environment {
    PRACTICE,
    COMPETITION,
    DEMO,
    ;
    private static Environment currentEnvironment = PRACTICE;
    private static final MappedSwitchableChooser<Environment> environmentChooser;
    static {
		var map = new LinkedHashMap<String, Environment>();
		map.put("Practice", PRACTICE);
		map.put("Competition", COMPETITION);
		map.put("Demo", DEMO);
		environmentChooser = new MappedSwitchableChooser<>(
			"Environment Chooser",
			map,
			PRACTICE
		);

        Logger.registerDashboardInput(new LoggedDashboardInput() {
            private static final SuppliedEdgeDetector fms_detector = new SuppliedEdgeDetector(DriverStation::isFMSAttached);
            private static final Alert fms_alert = new Alert("FMS detected, Competition Environment selected", AlertType.INFO);
            private static final Alert fms_no_comp_alert = new Alert("FMS detected but selected Environment is not Competition", AlertType.WARNING);
            private static final Alert demo_alert = new Alert("Demo Environment selected, Robot functionality restricted", AlertType.WARNING);
            public void periodic() {
                fms_detector.update();
                if(fms_detector.risingEdge()) {
                    environmentChooser.setSelected(COMPETITION);
                }
                currentEnvironment = environmentChooser.get();
                environmentChooser.setActive(currentEnvironment);
                fms_alert.set(fms_detector.getValue() && isCompetition());
                fms_no_comp_alert.set(fms_detector.getValue() && !isCompetition());
                demo_alert.set(isDemo());
            }
        });
    }

    public static Environment getEnvironment() {
        return currentEnvironment;
    }
    public static boolean is(Environment is) {
        return is.equals(currentEnvironment);
    }
    public static boolean isPractice() {
        return is(PRACTICE);
    }
    public static boolean isCompetition() {
        return is(COMPETITION);
    }
    public static boolean isDemo() {
        return is(DEMO);
    }

    public static DoubleSupplier switchVar(DoubleSupplier prac_comp, DoubleSupplier demo) {
        return switchVar(prac_comp, prac_comp, demo);
    }
    public static DoubleSupplier switchVar(DoubleSupplier prac, DoubleSupplier comp, DoubleSupplier demo) {
        return () -> switch(currentEnvironment) {
            default -> prac.getAsDouble();
            case COMPETITION -> comp.getAsDouble();
            case DEMO -> demo.getAsDouble();
        };
    }
    public static <T> Supplier<T> switchVar(Supplier<T> prac_comp, Supplier<T> demo) {
        return switchVar(prac_comp, prac_comp, demo);
    }
    public static <T> Supplier<T> switchVar(Supplier<T> prac, Supplier<T> comp, Supplier<T> demo) {
        return () -> switch(currentEnvironment) {
            default -> prac.get();
            case COMPETITION -> comp.get();
            case DEMO -> demo.get();
        };
    }
}
