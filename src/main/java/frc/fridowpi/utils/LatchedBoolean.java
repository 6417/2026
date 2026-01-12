package frc.fridowpi.utils;

public interface LatchedBoolean {
    default boolean updateAndGet(boolean val) {
        update(val);
        return  get();
    }

    boolean get();
    void update(boolean val);
}
