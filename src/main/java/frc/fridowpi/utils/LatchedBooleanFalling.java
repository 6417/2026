package frc.fridowpi.utils;

public class LatchedBooleanFalling implements LatchedBoolean{
    private boolean previous;
    private boolean currentState = false;
    public LatchedBooleanFalling(boolean initial) {
        previous = initial;
    }

    @Override
    public boolean get() {
        return currentState;
    }

    @Override
    public void update(boolean val) {
        currentState = previous && !val;
        previous = val;
    }
}
