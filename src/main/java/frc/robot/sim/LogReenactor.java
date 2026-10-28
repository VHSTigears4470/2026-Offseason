package frc.robot.sim;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.wpilib.datalog.DataLogIterator;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.hardware.hal.simulation.DriverStationDataJNI;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.simulation.DriverStationSim;

import box3dsim.arena.Alliance;
import box3dsim.math.Pose3;
import box3dsim.season.rebuilt2026.MatchPhase;
import box3dsim.season.rebuilt2026.RebuiltField;

import frc.robot.RobotContainer;

/**
 * Re-enacts an AdvantageKit log in the physics sim. The log's driver station (enable, mode, alliance, match time, game
 * data, joysticks) is written into HAL each loop, so the robot code drives the simulated robot as the real driver
 * did. The simulated pose estimate is compared with the logged Drive/Pose while enabled. Writes
 * build/reenact-results.json and exits 0, or 1 when the log can't be used.
 */
public final class LogReenactor {
    private static final double LOOP = 0.02;
    /** Starts this long before the first enable, so the robot code sees the DS settle first. */
    private static final double LEAD = 2;
    /** Runs on this long after the last disable. */
    private static final double TAIL = 1;
    private static final String DS = "/DriverStation/";
    private static final String POSE = "/RealOutputs/Drive/Pose";
    private static final String LOOP_TIMESTAMP = "/Timestamp";
    private static final int JOYSTICKS = 6;
    /** A logged start pose this close to a field edge is odometry that was never seeded. */
    private static final double ON_FIELD_MARGIN = 0.3;
    /** Logged pose changes within one loop larger than these are odometry resets. */
    private static final double JUMP_DISTANCE = 0.3;
    private static final double JUMP_DEGREES = 20;

    private final RobotSim sim;
    private final RobotContainer container;
    private final String logPath;
    private final Map<String, Series> series;
    private final long firstEnable;
    private final long startMicros;
    private final long endMicros;
    private final boolean fmsMatch;
    private final long wallStart = System.nanoTime();

    private int loops;
    private boolean placed;
    private int enabledSamples;
    private double sumSquaredError;
    private double maxError;
    private double lastError = Double.NaN;
    private double autoEndError = Double.NaN;
    private boolean prevAuto;
    private double loggedPath;
    private double simPath;
    private Pose2d prevLogged;
    private Pose2d prevSim;
    /** Logged poses map to the field by the transform logAnchor → simAnchor; both null when they already match. */
    private Pose2d simAnchor;
    private Pose2d logAnchor;
    private Pose2d prevRaw;
    private boolean jumped;

    /** Sample-and-hold values of one log entry. */
    private record Series(long[] times, Object[] values) {
        Object at(long t) {
            int i = Arrays.binarySearch(times, t);
            if (i < 0) {
                i = -i - 2;
            }
            return i < 0 ? null : values[i];
        }
    }

    /** Where the re-enacted run is logged: logs/<source name>_reenact.wpilog. */
    public static String outputPath() {
        String name = Path.of(requiredLogPath()).getFileName().toString().replaceFirst("\\.wpilog$", "");
        return Path.of("logs", name + "_reenact.wpilog").toString();
    }

    private static String requiredLogPath() {
        String p = System.getenv("SIM_LOG");
        if (p == null || p.isBlank()) {
            System.out.println("REENACT RESULT: FAIL no log; pass -PsimLog=<path to .wpilog>");
            System.exit(1);
        }
        return p.trim();
    }

    public LogReenactor(RobotSim sim, RobotContainer container) {
        this.sim = sim;
        this.container = container;
        this.logPath = requiredLogPath();
        this.series = read(logPath);

        Series enabled = series.get(DS + "Enabled");
        long enableAt = -1;
        long lastDisable = -1;
        if (enabled != null) {
            boolean prev = false;
            for (int i = 0; i < enabled.times().length; i++) {
                boolean on = (Boolean) enabled.values()[i];
                if (on && !prev && enableAt < 0) enableAt = enabled.times()[i];
                if (!on && prev) lastDisable = enabled.times()[i];
                prev = on;
            }
            if (prev) lastDisable = lastTimestamp();
        }
        firstEnable = enableAt;
        if (firstEnable < 0) {
            System.out.println("REENACT RESULT: FAIL " + logPath + " has no enabled period");
            System.exit(1);
        }
        startMicros = Math.max(enabled.times()[0], firstEnable - (long) (LEAD * 1e6));
        endMicros = Math.min(lastTimestamp(), lastDisable + (long) (TAIL * 1e6));
        fmsMatch = Boolean.TRUE.equals(at("FMSAttached", firstEnable));
        // A practice log has no 2:40 match clock to follow, so outputs are never cut.
        sim.setForceFreePlay(!fmsMatch);
        if (!series.containsKey(POSE)) {
            System.out.println("[reenact] " + POSE + " is not in the log; pose is compared from the default start");
        }
        String auto = System.getenv("SIM_AUTO");
        if (auto == null || auto.isBlank()) {
            System.out.println("[reenact] no -PsimAuto given: AUTO runs the chooser default (None)");
        }
        System.out.printf(Locale.ROOT, "[reenact] %s: %.1f s from %.1f s, %s%n", logPath,
                (endMicros - startMicros) / 1e6, startMicros / 1e6, fmsMatch ? "FMS match" : "practice (free play)");
    }

    private long lastTimestamp() {
        long last = 0;
        for (Series s : series.values()) {
            last = Math.max(last, s.times()[s.times().length - 1]);
        }
        return last;
    }

    private static Map<String, Series> read(String path) {
        DataLogReader reader;
        try {
            reader = new DataLogReader(path);
        } catch (IOException e) {
            System.out.println("REENACT RESULT: FAIL cannot read " + path + ": " + e.getMessage());
            System.exit(1);
            return Map.of();
        }
        if (!reader.isValid()) {
            System.out.println("REENACT RESULT: FAIL " + path + " is not a WPILOG file");
            System.exit(1);
        }
        Map<Integer, String[]> entries = new HashMap<>();
        Map<String, List<Long>> times = new HashMap<>();
        Map<String, List<Object>> values = new HashMap<>();
        int loopEntry = -1;
        List<Long> loopTimes = new ArrayList<>();
        DataLogIterator records = reader.iterator();
        while (records.hasNext()) {
            DataLogRecord r;
            try {
                r = records.next();
            } catch (RuntimeException e) {
                // A robot powered off mid-write leaves the last record cut short.
                System.out.println("[reenact] log ends in a truncated record; using what precedes it");
                break;
            }
            if (r.isStart()) {
                DataLogRecord.StartRecordData d = r.getStartData();
                if (d.name.startsWith(DS) || d.name.equals(POSE)) {
                    entries.put(d.entry, new String[] {d.name, d.type});
                } else if (d.name.equals(LOOP_TIMESTAMP)) {
                    loopEntry = d.entry;
                }
                continue;
            }
            if (r.isControl()) {
                continue;
            }
            if (r.getEntry() == loopEntry) {
                loopTimes.add(r.getTimestamp());
                continue;
            }
            String[] e = entries.get(r.getEntry());
            if (e == null) {
                continue;
            }
            Object v = switch (e[1]) {
                case "boolean" -> r.getBoolean();
                case "int64" -> r.getInteger();
                case "double" -> r.getDouble();
                case "string" -> r.getString();
                case "float[]" -> r.getFloatArray();
                case "int64[]" -> r.getIntegerArray();
                case "struct:Pose2d" -> {
                    ByteBuffer b = r.getRawBuffer().order(ByteOrder.LITTLE_ENDIAN);
                    yield new Pose2d(b.getDouble(0), b.getDouble(8), new Rotation2d(b.getDouble(16)));
                }
                default -> null;
            };
            if (v != null) {
                times.computeIfAbsent(e[0], k -> new ArrayList<>()).add(r.getTimestamp());
                values.computeIfAbsent(e[0], k -> new ArrayList<>()).add(v);
            }
        }
        long ticksPerMicro = ticksPerMicro(loopTimes);
        Map<String, Series> out = new HashMap<>();
        for (var t : times.entrySet()) {
            out.put(t.getKey(), new Series(t.getValue().stream().mapToLong(v -> v / ticksPerMicro).toArray(),
                    values.get(t.getKey()).toArray()));
        }
        return out;
    }

    /** Record timestamp ticks per µs, from AdvantageKit's once-per-loop entry being 20 ms apart. */
    private static long ticksPerMicro(List<Long> loopTimes) {
        if (loopTimes.size() < 10) {
            return 1;
        }
        long[] deltas = new long[loopTimes.size() - 1];
        for (int i = 0; i < deltas.length; i++) {
            deltas[i] = loopTimes.get(i + 1) - loopTimes.get(i);
        }
        Arrays.sort(deltas);
        double perMicro = deltas[deltas.length / 2] / (LOOP * 1e6);
        return perMicro > 30 ? 1000 : 1;
    }

    private Object at(String dsKey, long t) {
        Series s = series.get(DS + dsKey);
        return s == null ? null : s.at(t);
    }

    private boolean bool(String key, long t) {
        return Boolean.TRUE.equals(at(key, t));
    }

    private long integer(String key, long t) {
        Object v = at(key, t);
        return v == null ? 0 : (Long) v;
    }

    private String string(String key, long t) {
        Object v = at(key, t);
        return v == null ? "" : (String) v;
    }

    public void periodic() {
        long now = startMicros + Math.round(loops * LOOP * 1e6);
        loops++;
        if (!placed) {
            placed = true;
            placeRobot(loggedPose(now));
        }
        measure(now);
        if (now >= endMicros) {
            finish(now);
            return;
        }
        // Read by the robot code at the start of the next loop.
        applyDriverStation(now + Math.round(LOOP * 1e6));
    }

    private Pose2d loggedPose(long t) {
        Series s = series.get(POSE);
        return s == null ? null : (Pose2d) s.at(t);
    }

    /**
     * Starts the robot at the logged pose when that is a place on the field. Odometry that was never seeded (no
     * vision, no auto reset) starts near the origin instead; then the robot starts at its alliance's start pose and
     * logged poses are compared relative to where they started.
     */
    private void placeRobot(Pose2d logged) {
        if (logged == null || onField(logged)) {
            if (logged != null) {
                sim.placeRobot(Pose3.ofXYYaw(logged.getX(), logged.getY(), 0, logged.getRotation().getRadians()));
            }
            return;
        }
        long station = integer("AllianceStation", firstEnable);
        Pose3 start = SimConstants.startPose(station >= 1 && station <= 3 ? Alliance.RED : Alliance.BLUE);
        sim.placeRobot(start);
        simAnchor = PoseConversions.toPose2d(start);
        logAnchor = logged;
        System.out.printf(Locale.ROOT, "[reenact] logged start pose (%.2f, %.2f) is off the field (odometry never"
                + " seeded); starting at the alliance start pose and comparing relative motion%n",
                logged.getX(), logged.getY());
    }

    private static boolean onField(Pose2d p) {
        return p.getX() > ON_FIELD_MARGIN && p.getX() < RebuiltField.LENGTH - ON_FIELD_MARGIN
                && p.getY() > ON_FIELD_MARGIN && p.getY() < RebuiltField.WIDTH - ON_FIELD_MARGIN;
    }

    /**
     * The logged pose in field coordinates. A jump in the logged pose (an odometry reset) re-anchors: to the field when
     * the new pose is on it, else so the field pose carries on from where it was.
     */
    private Pose2d alignLogged(Pose2d raw) {
        if (prevRaw != null && isJump(prevRaw, raw)) {
            if (onField(raw)) {
                simAnchor = null;
                logAnchor = null;
            } else {
                simAnchor = toField(prevRaw);
                logAnchor = raw;
            }
            jumped = true;
        }
        prevRaw = raw;
        return toField(raw);
    }

    private Pose2d toField(Pose2d raw) {
        return simAnchor == null ? raw : simAnchor.transformBy(new Transform2d(logAnchor, raw));
    }

    private static boolean isJump(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation()) > JUMP_DISTANCE
                || Math.abs(a.getRotation().minus(b.getRotation()).getDegrees()) > JUMP_DEGREES;
    }

    private void applyDriverStation(long t) {
        DriverStationSim.setDsAttached(bool("DSAttached", t));
        DriverStationSim.setFmsAttached(bool("FMSAttached", t));
        DriverStationSim.setEnabled(bool("Enabled", t));
        String mode = string("RobotMode", t);
        DriverStationSim.setRobotMode(mode.isEmpty() ? RobotMode.UNKNOWN : RobotMode.valueOf(mode));
        DriverStationSim.setAllianceStationId(switch ((int) integer("AllianceStation", t)) {
            case 1 -> AllianceStationID.RED_1;
            case 2 -> AllianceStationID.RED_2;
            case 3 -> AllianceStationID.RED_3;
            case 4 -> AllianceStationID.BLUE_1;
            case 5 -> AllianceStationID.BLUE_2;
            case 6 -> AllianceStationID.BLUE_3;
            default -> AllianceStationID.UNKNOWN;
        });
        Object matchTime = at("MatchTime", t);
        DriverStationSim.setMatchTime(matchTime == null ? -1 : (Double) matchTime);
        String gameData = string("GameData", t);
        DriverStationSim.setGameData(gameData);
        applyGameData(gameData);

        for (int j = 0; j < JOYSTICKS; j++) {
            String k = "Joystick" + j + "/";
            DriverStationSim.setJoystickName(j, string(k + "Name", t));
            DriverStationSim.setJoystickGamepadType(j, (int) integer(k + "Type", t));
            DriverStationSim.setJoystickIsGamepad(j, bool(k + "IsGamepad", t));
            DriverStationSim.setJoystickSupportedOutputs(j, (int) integer(k + "SupportedOutputs", t));
            DriverStationSim.setJoystickButtonsAvailable(j, integer(k + "ButtonsAvailable", t));
            DriverStationDataJNI.setJoystickButtonsValue(j, integer(k + "ButtonValues", t));
            DriverStationSim.setJoystickPOVsAvailable(j, (int) integer(k + "POVsAvailable", t));
            if (at(k + "POVValues", t) instanceof long[] povs) {
                for (int i = 0; i < povs.length; i++) {
                    DriverStationDataJNI.setJoystickPOV(j, i, (byte) povs[i]);
                }
            }
            DriverStationSim.setJoystickAxesAvailable(j, (int) integer(k + "AxesAvailable", t));
            if (at(k + "AxisValues", t) instanceof float[] axes) {
                for (int i = 0; i < axes.length; i++) {
                    DriverStationSim.setJoystickAxis(j, i, axes[i]);
                }
            }
        }
        DriverStationSim.notifyNewData();
    }

    /** FMS names the alliance whose HUB goes inactive first a few seconds after AUTO. */
    private void applyGameData(String gameData) {
        MatchPhase phase = sim.match().phase();
        if (phase == MatchPhase.AUTO || phase == MatchPhase.FREE_PLAY || phase == MatchPhase.POST_MATCH) {
            return;
        }
        if (gameData.startsWith("R")) {
            sim.match().setFirstInactive(Alliance.RED);
        } else if (gameData.startsWith("B")) {
            sim.match().setFirstInactive(Alliance.BLUE);
        }
    }

    private void measure(long t) {
        Pose2d raw = loggedPose(t);
        Pose2d simPose = container.getDriveSubsystem().getEstimatedPosition();
        Logger.recordOutput("Reenact/LogTimeS", t / 1e6);
        if (raw == null) {
            return;
        }
        jumped = false;
        Pose2d logged = alignLogged(raw);
        double error = logged.getTranslation().getDistance(simPose.getTranslation());
        double headingError = Math.abs(logged.getRotation().minus(simPose.getRotation()).getDegrees());
        Logger.recordOutput("Reenact/LoggedPose", logged);
        Logger.recordOutput("Reenact/LoggedPoseRaw", raw);
        Logger.recordOutput("Reenact/PoseErrorM", error);
        Logger.recordOutput("Reenact/HeadingErrorDeg", headingError);

        boolean enabled = bool("Enabled", t);
        boolean auto = enabled && "AUTONOMOUS".equals(string("RobotMode", t));
        if (prevAuto && !auto) {
            autoEndError = error;
        }
        prevAuto = auto;
        if (enabled) {
            enabledSamples++;
            sumSquaredError += error * error;
            maxError = Math.max(maxError, error);
            lastError = error;
            if (prevLogged != null && !jumped) {
                loggedPath += logged.getTranslation().getDistance(prevLogged.getTranslation());
                simPath += simPose.getTranslation().getDistance(prevSim.getTranslation());
            }
            prevLogged = logged;
            prevSim = simPose;
        } else {
            prevLogged = null;
        }
    }

    private void finish(long t) {
        double rms = enabledSamples == 0 ? Double.NaN : Math.sqrt(sumSquaredError / enabledSamples);
        double simSeconds = (t - startMicros) / 1e6;
        double wall = (System.nanoTime() - wallStart) / 1e9;
        String json = String.format(Locale.ROOT, """
                {
                  "log": "%s",
                  "output": "%s",
                  "fmsMatch": %b,
                  "auto": "%s",
                  "simSeconds": %.2f, "wallSeconds": %.2f,
                  "enabledSeconds": %.2f,
                  "poseErrorM": {"rms": %.3f, "max": %.3f, "final": %.3f, "autoEnd": %.3f},
                  "pathLengthM": {"logged": %.2f, "sim": %.2f, "ratio": %.3f},
                  "blueScore": %d, "redScore": %d, "shotsFired": %d, "intakeTaken": %d
                }
                """, logPath.replace("\\", "/"), outputPath().replace("\\", "/"), fmsMatch,
                String.valueOf(System.getenv("SIM_AUTO")).replace("\"", "'"), simSeconds, wall, enabledSamples * LOOP,
                rms, maxError, lastError, autoEndError, loggedPath, simPath,
                loggedPath > 0 ? simPath / loggedPath : Double.NaN, sim.match().score(Alliance.BLUE),
                sim.match().score(Alliance.RED), sim.shotsFired(), sim.intakeAcquired());
        try {
            Files.createDirectories(Path.of("build"));
            Files.writeString(Path.of("build", "reenact-results.json"), json);
        } catch (IOException e) {
            System.out.println("[reenact] could not write build/reenact-results.json: " + e.getMessage());
        }
        System.out.printf(Locale.ROOT,
                "REENACT RESULT: %.0f s enabled, pose error rms %.2f m max %.2f m final %.2f m, path sim/log %.3f%n",
                enabledSamples * LOOP, rms, maxError, lastError, loggedPath > 0 ? simPath / loggedPath : Double.NaN);
        Logger.end();
        System.exit(0);
    }
}
