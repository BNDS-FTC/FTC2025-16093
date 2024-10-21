package XCYOS;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class XCYOSCore {
    private static final ArrayList<Task> registeredTask = new ArrayList<>();
    private static final ArrayList<Task> newTasks = new ArrayList<>(3);
    private static OpMode opmode;
    private static List<LynxModule> allHubs;
    private static Component[] allComponents;
    private static long lastTime;

    public static boolean isNotConflict(Task task) {
        for (Task registered : registeredTask) {
            if (registered == task) continue;
            for (Component c_registered : registered.getComponentsOccupied()) {
                for (Component c_task : task.getComponentsOccupied()) {
                    if (c_registered == c_task) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    public static void addTask(Task task) {
        for (Task registered : registeredTask) {
            for (Component c_registered : registered.getComponentsOccupied()) {
                for (Component c_task : task.getComponentsOccupied()) {
                    if (c_registered == c_task) {
                        if (registered.getPriority() < task.getPriority()) {
                            registered.setStatus(Task.Status.ENDED);
                            registeredTask.add(task);
                        } else {
                            logForTime("[XCYOS] conflict task not added", 500);
                            return;
                        }
                    }
                }
            }
        }
        registeredTask.add(task);
    }

    public static void logForTime(String msg, int time) {
        registeredTask.add(new Task() {
            long endTime;

            @Override
            public void setUp() {
                endTime = System.currentTimeMillis() + time;
            }

            @Override
            public void run() {
                if (endTime > System.currentTimeMillis()) {
                    log(msg);
                } else {
                    status = Status.ENDED;
                }
            }
        });
    }

    public static void setUp(OpMode opmode, Component... components) {
        lastTime = System.currentTimeMillis();
        XCYOSCore.opmode = opmode;
        allComponents = components;
        for (Component c : allComponents) {
            c.setUp(opmode.hardwareMap);
        }
        LynxModuleUtil.ensureMinimumFirmwareVersion(opmode.hardwareMap);
        allHubs = opmode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        log("[XCYOS] init complete");
        update();
    }

    public static void log(String msg) {
        opmode.telemetry.addLine(msg);
    }

    public static Telemetry getTelemetry() {
        return opmode.telemetry;
    }

    public static void freeComponent(Component component) {

    }

    public static void update() {
        long currentTime = System.currentTimeMillis();
        opmode.telemetry.addData("[XCYOS] rate", (double) 1 / (lastTime - currentTime) * 1000);
        lastTime = currentTime;
        getTelemetry().addData("[XCYOS] task count", registeredTask.size());
        Iterator<Task> iter = registeredTask.iterator();
        while (iter.hasNext()) {
            Task task = iter.next();
            if (task.type == Task.Type.INSTANT) {
                switch (task.status) {
                    case ENDED:
                        task.end();
                        iter.remove();
                        newTasks.addAll(task.getNextTasks());
                    case PENDING:
                        task.setUp();
                        task.setStatus(Task.Status.RUNNING);
                    case RUNNING:
                        task.run();
                }
            } else if (task.type == Task.Type.BASE) {
                if (task.status != Task.Status.RUNNING && isNotConflict(task)) {
                    task.setStatus(Task.Status.RUNNING);
                } else if (task.status == Task.Status.RUNNING) {
                    task.run();
                } else {
                    log("suspend");
                }
            }
        }
        registeredTask.addAll(newTasks);
        newTasks.clear();
        for (Component c : allComponents) {
            c.update();
        }
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        opmode.telemetry.update();
    }
}
