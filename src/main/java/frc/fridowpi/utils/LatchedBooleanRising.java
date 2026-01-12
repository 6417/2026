package frc.fridowpi.utils;

public class LatchedBooleanRising implements LatchedBoolean{
    private boolean previous;
    private boolean currentState = false;
    public LatchedBooleanRising(boolean initial) {
        previous = initial;
    }

    @Override
    public boolean get() {
        return currentState;
    }

    @Override
    public void update(boolean val) {
        currentState = !previous && val;
        previous = val;
    }
}
