"""SQLite WAL-mode database writer with batched inserts."""

import os
import sqlite3
import threading
import time
from collections import deque
from pathlib import Path


class DbWriter:
    def __init__(self, db_path: str, retention_days: int = 30, logger=None):
        self._db_path = os.path.expanduser(db_path)
        self._retention_days = retention_days
        self._logger = logger
        self._queue: deque[tuple[str, tuple]] = deque()
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._conn: sqlite3.Connection | None = None

    # -- lifecycle --

    def start(self):
        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._init_schema()
        self._running = True
        self._thread = threading.Thread(target=self._flush_loop, daemon=True)
        self._thread.start()
        if self._logger:
            self._logger.info(f"DB opened: {self._db_path}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        self._flush_now()
        if self._conn:
            self._conn.close()

    # -- public API --

    def log_command(self, robot_id: str, endpoint: str, params: str,
                    result: str, duration_ms: float):
        self._enqueue(
            "INSERT INTO command_log(robot_id,endpoint,params,result,duration_ms) "
            "VALUES(?,?,?,?,?)",
            (robot_id, endpoint, params, result, duration_ms),
        )

    def log_nav_start(self, robot_id: str, goal_x: float, goal_y: float,
                      goal_theta: float) -> None:
        self._enqueue(
            "INSERT INTO navigation_log(robot_id,goal_x,goal_y,goal_theta,status) "
            "VALUES(?,?,?,?,?)",
            (robot_id, goal_x, goal_y, goal_theta, "active"),
        )

    def log_nav_end(self, robot_id: str, result: str, distance: float) -> None:
        self._enqueue(
            "UPDATE navigation_log SET ended_at=strftime('%Y-%m-%dT%H:%M:%f','now'),"
            "result=?,distance=?,status=? WHERE robot_id=? AND status='active'",
            (result, distance, result, robot_id),
        )

    def log_sensor_snapshot(self, robot_id: str, data: dict) -> None:
        self._enqueue(
            "INSERT INTO sensor_snapshot"
            "(robot_id,pose_x,pose_y,pose_yaw,battery,linear_vel,angular_vel,nav_status) "
            "VALUES(?,?,?,?,?,?,?,?)",
            (
                robot_id,
                data.get("pose_x"),
                data.get("pose_y"),
                data.get("pose_yaw"),
                data.get("battery"),
                data.get("linear_vel"),
                data.get("angular_vel"),
                data.get("nav_status"),
            ),
        )

    def log_event(self, robot_id: str, level: str, source: str,
                  message: str) -> None:
        self._enqueue(
            "INSERT INTO event_log(robot_id,level,source,message) VALUES(?,?,?,?)",
            (robot_id, level, source, message),
        )

    def query_commands(self, robot_id: str, limit: int = 50,
                       since: str | None = None) -> list[dict]:
        if since:
            return self._query(
                "SELECT id,ts,endpoint,params,result,duration_ms FROM command_log "
                "WHERE robot_id=? AND ts>=? ORDER BY ts DESC LIMIT ?",
                (robot_id, since, limit),
            )
        return self._query(
            "SELECT id,ts,endpoint,params,result,duration_ms FROM command_log "
            "WHERE robot_id=? ORDER BY ts DESC LIMIT ?",
            (robot_id, limit),
        )

    def query_navigation(self, robot_id: str, limit: int = 20) -> list[dict]:
        return self._query(
            "SELECT * FROM navigation_log WHERE robot_id=? "
            "ORDER BY started_at DESC LIMIT ?",
            (robot_id, limit),
        )

    def query_events(self, robot_id: str, limit: int = 100,
                     level: str | None = None,
                     since: str | None = None) -> list[dict]:
        clauses = ["robot_id=?"]
        params: list = [robot_id]
        if level:
            clauses.append("level=?")
            params.append(level)
        if since:
            clauses.append("ts>=?")
            params.append(since)
        where = " WHERE " + " AND ".join(clauses)
        params.append(limit)
        return self._query(
            f"SELECT id,ts,level,source,message FROM event_log{where} "
            "ORDER BY ts DESC LIMIT ?",
            tuple(params),
        )

    # -- internal --

    def _enqueue(self, sql: str, params: tuple):
        with self._lock:
            self._queue.append((sql, params))

    def _flush_loop(self):
        while self._running:
            time.sleep(1.0)
            self._flush_now()
            self._maybe_prune()

    def _flush_now(self):
        with self._lock:
            batch = list(self._queue)
            self._queue.clear()
        if not batch or not self._conn:
            return
        try:
            cur = self._conn.cursor()
            for sql, params in batch:
                cur.execute(sql, params)
            self._conn.commit()
        except Exception as exc:
            if self._logger:
                self._logger.error(f"DB flush error: {exc}")

    def _maybe_prune(self):
        if not self._conn or self._retention_days <= 0:
            return
        try:
            cutoff = f"-{self._retention_days} days"
            self._conn.execute(
                "DELETE FROM sensor_snapshot WHERE ts < datetime('now', ?)",
                (cutoff,),
            )
            self._conn.commit()
        except Exception:
            pass

    def _query(self, sql: str, params: tuple) -> list[dict]:
        if not self._conn:
            return []
        try:
            cur = self._conn.execute(sql, params)
            cols = [d[0] for d in cur.description]
            return [dict(zip(cols, row)) for row in cur.fetchall()]
        except Exception as exc:
            if self._logger:
                self._logger.error(f"DB query error: {exc}")
            return []

    def _init_schema(self):
        schema_path = Path(__file__).parent / "schema.sql"
        schema = schema_path.read_text()
        self._conn.executescript(schema)
        self._conn.commit()
