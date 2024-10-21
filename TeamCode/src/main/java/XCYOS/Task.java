package XCYOS;

import java.util.ArrayList;

public abstract class Task {
    protected Type type = Type.INSTANT;
    protected Status status = Status.PENDING;
    private final ArrayList<Task> nextTask = new ArrayList<>(1);
    private Task threadEnd = this;
    private Component[] componentsOccupied = new Component[0];
    private int priority = 1;

    public void setPriority(int priority) {
        this.priority = priority;
    }

    public int getPriority() {
        return priority;
    }

    public void setComponentsOccupied(Component... componentsOccupied) {
        this.componentsOccupied = componentsOccupied;
    }

    public Component[] getComponentsOccupied() {
        return componentsOccupied;
    }

    public void setUp() {

    }

    abstract public void run();

    public void end() {

    }

    protected void setEndTask(Task endTask) {
        threadEnd = endTask;
    }

    protected Task getEndTask() {
        return threadEnd;
    }

    protected void addNextTask(Task task) {
        nextTask.add(task);
    }

    protected ArrayList<Task> getNextTasks() {
        return nextTask;
    }

    public void setStatus(Status status) {
        this.status = status;
    }

    public void setType(Type type) {
        this.type = type;
    }

    public enum Type {
        BASE, INSTANT
    }

    public enum Status {
        PENDING, RUNNING, ENDED
    }
}
