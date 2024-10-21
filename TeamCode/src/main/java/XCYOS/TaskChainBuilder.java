package XCYOS;

import java.util.function.BooleanSupplier;

public class TaskChainBuilder {
    private Task base;
    private Task branchingTask;
    private Component[] component;
    private int priority;

    public TaskChainBuilder(){
        component = new Component[0];
        priority = 1;
        base = null;
        branchingTask = null;
    }

    public TaskChainBuilder(int priority, Component... component){
        this.component=component;
        this.priority=priority;
        base = null;
        branchingTask = null;
    }
    public TaskChainBuilder setPriority(int priority) {
        this.priority = priority;
        return this;
    }

    public TaskChainBuilder sleep(int time) {
        add(new DelayTask(time));
        return this;
    }

    public TaskChainBuilder add(Task task) {
        task.setComponentsOccupied(component);
        task.setPriority(priority);
        if (base == null) {
            base = task;
        } else {
            base.getEndTask().addNextTask(task);
        }
        task.setStatus(Task.Status.PENDING);
        base.setEndTask(task.getEndTask());
        return this;
    }

    public TaskChainBuilder add(Runnable runnable) {
        add(new Task() {
            @Override
            public void run() {
                runnable.run();
                status = Status.ENDED;
            }
        });
        return this;
    }

    public TaskChainBuilder waitFor(BooleanSupplier booleanSupplier) {
        add(new Task() {
            @Override
            public void run() {
                if (booleanSupplier.getAsBoolean()) {
                    status = Status.ENDED;
                }
            }
        });
        return this;
    }

    public TaskChainBuilder addBranch(Task task) {
        if (branchingTask != null) {
            throw new IllegalArgumentException("create new branch when last branch not end");
        }
        branchingTask = base.getEndTask();
        base.getEndTask().addNextTask(task);
        base.setEndTask(task.getEndTask());
        return this;
    }

    public Task getBase() {
        return base;
    }

    public TaskChainBuilder setComponent(Component... c) {
        component = c;
        return this;
    }

    public TaskChainBuilder end() {
        if (branchingTask != null) {
            base.setEndTask(branchingTask);
        }
        return this;
    }
}
