package frc.util.loggerUtil;

import java.util.Optional;
import java.util.function.IntFunction;

public class LoggerUtil {
    
    public static <T> T[] toArray(Optional<T> optional, IntFunction<T[]> generator) {
        T[] array;
        if (optional.isPresent()) {
            array = generator.apply(1);
            array[0] = optional.get();
        } else {
            array = generator.apply(0);
        }
        return array;
    }
}
