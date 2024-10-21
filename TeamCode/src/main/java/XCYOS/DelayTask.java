package XCYOS;

public class DelayTask extends Task {
    private long timeEnd;
    private final long timeDelay;

    public DelayTask(int time_ms) {
        timeDelay = time_ms;
    }

    @Override
    public void setUp() {
        timeEnd = System.currentTimeMillis() + timeDelay;
    }

    @Override
    public void run() {
        if (timeEnd < System.currentTimeMillis()) {
            status = Status.ENDED;
        }
    }
}
