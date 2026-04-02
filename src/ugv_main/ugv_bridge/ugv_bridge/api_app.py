"""FastAPI application with multi-robot REST endpoints."""

import base64
import json
import time
import logging

import httpx
from fastapi import FastAPI, Depends, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.staticfiles import StaticFiles
from jwt import PyJWKClient, decode as jwt_decode

from .shared_state import RobotState
from .ros_interface import RosInterface
from .db_writer import DbWriter

logger = logging.getLogger(__name__)


class RobotHandle:
    """Bundle of per-robot objects."""
    __slots__ = ("robot_id", "state", "ros_if")

    def __init__(self, robot_id: str, state: RobotState,
                 ros_if: RosInterface):
        self.robot_id = robot_id
        self.state = state
        self.ros_if = ros_if


def create_app(robots: dict[str, RobotHandle], db: DbWriter,
               static_dir: str = "", oauth2_jwks_uri: str = "") -> FastAPI:

    app = FastAPI(title="UGV Bridge API", version="0.2.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # -- OAuth2 JWT verification --

    security = HTTPBearer(auto_error=False)
    jwk_client = PyJWKClient(oauth2_jwks_uri) if oauth2_jwks_uri else None

    async def verify_token(
        cred: HTTPAuthorizationCredentials | None = Depends(security),
    ):
        if not jwk_client:
            return None  # OAuth2 disabled
        if not cred:
            raise JSONResponse(status_code=401, content={"error": "missing token"})
        try:
            signing_key = jwk_client.get_signing_key_from_jwt(cred.credentials)
            payload = jwt_decode(
                cred.credentials,
                signing_key.key,
                algorithms=["RS256"],
                options={"verify_aud": False},
            )
            return payload
        except Exception as e:
            logger.warning("JWT verification failed: %s", e)
            from fastapi import HTTPException
            raise HTTPException(status_code=401, detail="invalid token")

    # -- helper --

    def get_robot(robot_id: str) -> RobotHandle | None:
        return robots.get(robot_id)

    # -- command logging middleware --

    @app.middleware("http")
    async def log_commands(request: Request, call_next):
        if request.method == "POST" and "/api/" in request.url.path:
            t0 = time.monotonic()
            body_bytes = await request.body()

            async def receive():
                return {"type": "http.request", "body": body_bytes}
            request._receive = receive

            response = await call_next(request)
            duration_ms = (time.monotonic() - t0) * 1000

            # Extract robot_id from path: /api/{robot_id}/...
            parts = request.url.path.strip("/").split("/")
            rid = parts[1] if len(parts) >= 3 else "unknown"

            db.log_command(
                rid,
                request.url.path,
                body_bytes.decode(errors="replace")[:500],
                str(response.status_code),
                round(duration_ms, 1),
            )
            return response
        return await call_next(request)

    # -- health & robot list --

    @app.get("/api/health")
    async def health():
        return {"status": "ok", "robots": list(robots.keys())}

    @app.get("/api/robots")
    async def list_robots(_=Depends(verify_token)):
        result = []
        for rid, rh in robots.items():
            snap = rh.state.snapshot_full()
            result.append({"robot_id": rid, **snap})
        return {"robots": result}

    # -- per-robot endpoints --

    @app.get("/api/{robot_id}/status")
    async def status(robot_id: str, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        return rh.state.snapshot_full()

    @app.get("/api/{robot_id}/map")
    async def get_map(robot_id: str, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        png, meta = rh.state.snapshot_map_png()
        if png is None or meta is None:
            return JSONResponse({"error": "no map"}, 404)
        b64 = base64.b64encode(png).decode()
        return {
            "image": b64,
            "width": meta.width,
            "height": meta.height,
            "resolution": meta.resolution,
            "origin_x": meta.origin_x,
            "origin_y": meta.origin_y,
            "origin_yaw": meta.origin_yaw,
            "revision": rh.state.snapshot_map_revision(),
        }

    @app.post("/api/{robot_id}/navigate")
    async def navigate(robot_id: str, request: Request, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        body = await request.json()
        x = float(body.get("x", 0))
        y = float(body.get("y", 0))
        theta = float(body.get("theta", 0))
        ok = rh.ros_if.send_nav_goal(x, y, theta)
        if ok:
            return {"status": "goal_sent", "x": x, "y": y, "theta": theta}
        return JSONResponse({"error": "nav2 unavailable"}, 503)

    @app.post("/api/{robot_id}/cancel")
    async def cancel_nav(robot_id: str, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        ok = rh.ros_if.cancel_nav()
        return {"canceled": ok}

    @app.post("/api/{robot_id}/initial_pose")
    async def initial_pose(robot_id: str, request: Request, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        body = await request.json()
        rh.ros_if.publish_initial_pose(
            float(body.get("x", 0)),
            float(body.get("y", 0)),
            float(body.get("yaw", 0)),
        )
        return {"status": "ok"}

    @app.post("/api/{robot_id}/cmd_vel")
    async def cmd_vel(robot_id: str, request: Request, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        body = await request.json()
        rh.ros_if.publish_cmd_vel(
            float(body.get("linear", 0)),
            float(body.get("angular", 0)),
        )
        return {"status": "ok"}

    @app.post("/api/{robot_id}/arm")
    async def arm(robot_id: str, request: Request, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        body = await request.json()
        positions = [float(p) for p in body.get("positions", [0, 0, 0, 0])]
        rh.ros_if.publish_arm(positions)
        return {"status": "ok"}

    @app.post("/api/{robot_id}/gripper")
    async def gripper(robot_id: str, request: Request, _=Depends(verify_token)):
        rh = get_robot(robot_id)
        if not rh:
            return JSONResponse({"error": "unknown robot"}, 404)
        body = await request.json()
        rh.ros_if.publish_gripper(float(body.get("value", 0)))
        return {"status": "ok"}

    # -- logs (per-robot) --

    @app.get("/api/{robot_id}/logs")
    async def get_logs(
        robot_id: str,
        type: str = Query("command", pattern="^(command|event)$"),
        limit: int = Query(50, ge=1, le=500),
        since: str | None = Query(None),
        level: str | None = Query(None),
        _=Depends(verify_token),
    ):
        if robot_id not in robots:
            return JSONResponse({"error": "unknown robot"}, 404)
        if type == "command":
            return {"logs": db.query_commands(robot_id, limit, since)}
        return {"logs": db.query_events(robot_id, limit, level, since)}

    @app.get("/api/{robot_id}/logs/navigation")
    async def get_nav_logs(robot_id: str,
                           limit: int = Query(20, ge=1, le=200),
                           _=Depends(verify_token)):
        if robot_id not in robots:
            return JSONResponse({"error": "unknown robot"}, 404)
        return {"logs": db.query_navigation(robot_id, limit)}

    # -- static files (frontend) --

    if static_dir:
        app.mount("/", StaticFiles(directory=static_dir, html=True),
                  name="static")

    return app
