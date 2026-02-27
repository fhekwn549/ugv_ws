CREATE TABLE IF NOT EXISTS command_log (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%f', 'now')),
    robot_id    TEXT    NOT NULL,
    endpoint    TEXT    NOT NULL,
    params      TEXT,
    result      TEXT,
    duration_ms REAL
);

CREATE TABLE IF NOT EXISTS navigation_log (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id    TEXT    NOT NULL,
    goal_x      REAL,
    goal_y      REAL,
    goal_theta  REAL,
    started_at  TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%f', 'now')),
    ended_at    TEXT,
    result      TEXT,
    distance    REAL,
    status      TEXT    NOT NULL DEFAULT 'active'
);

CREATE TABLE IF NOT EXISTS sensor_snapshot (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%f', 'now')),
    robot_id    TEXT    NOT NULL,
    pose_x      REAL,
    pose_y      REAL,
    pose_yaw    REAL,
    battery     REAL,
    linear_vel  REAL,
    angular_vel REAL,
    nav_status  TEXT
);

CREATE TABLE IF NOT EXISTS event_log (
    id      INTEGER PRIMARY KEY AUTOINCREMENT,
    ts      TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%f', 'now')),
    robot_id TEXT   NOT NULL,
    level   TEXT    NOT NULL,
    source  TEXT    NOT NULL,
    message TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_command_log_ts ON command_log(robot_id, ts);
CREATE INDEX IF NOT EXISTS idx_navigation_log_started ON navigation_log(robot_id, started_at);
CREATE INDEX IF NOT EXISTS idx_sensor_snapshot_ts ON sensor_snapshot(robot_id, ts);
CREATE INDEX IF NOT EXISTS idx_event_log_ts ON event_log(robot_id, ts);
