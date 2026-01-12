package frc.fridowpi.utils;

import java.util.AbstractMap;
import java.util.Map.Entry;
import java.util.function.Function;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Algorithms {
	public static <K, V, RV> Function<Entry<K, V>, Entry<K, RV>> mapEntryFunction(Function<V, RV> function) {
        return (entry) -> new AbstractMap.SimpleEntry<>(entry.getKey(), function.apply(entry.getValue()));
    }

    public static Function<SwerveModuleState, SwerveModuleState> mapSwerveModuleStateSpeed(Function<Double, Double> function) {
        return (state) -> new SwerveModuleState(function.apply(state.speedMetersPerSecond), state.angle);
    }
}
