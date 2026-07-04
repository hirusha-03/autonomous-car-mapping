const API = "http://127.0.0.1:8000";
const canvas = document.getElementById("grid");
const ctx = canvas.getContext("2d");

const GRID_SIZE = 50;
const CELL_SIZE = canvas.width / GRID_SIZE;

// Direction name (from backend) → unit vector in screen space (x right, y down)
const DIR_SCREEN_VECTOR = {
    East:  [1, 0],
    North: [0, -1],
    West:  [-1, 0],
    South: [0, 1],
};
const DIR_ORDER = ["East", "North", "West", "South"]; // must match backend DIRECTION_NAMES order

// Must match sensorsweep.py's CELL_SIZE_CM — converts a sensor's raw cm
// reading into grid-cell units so the ray can be drawn at the right length.
const CELL_SIZE_CM = 30;

const SENSOR_RAY_COLOR = { front: "#00e5ff", left: "#76ff03", right: "#ff4081" };

// Direction a sensor ray points in screen space. Front is straight ahead;
// left/right mirror sensorsweep.py's _diagonal_vector (forward + perpendicular)
// approximating the ~30deg-angled side sensors on a 4-directional grid.
function sensorRayVector(directionName, side) {
    const [dx, dy] = DIR_SCREEN_VECTOR[directionName];
    if (!side) return [dx, dy];
    const idx = DIR_ORDER.indexOf(directionName);
    const perpIdx = side === "left" ? (idx + 1) % 4 : (idx + 3) % 4;
    const [pdx, pdy] = DIR_SCREEN_VECTOR[DIR_ORDER[perpIdx]];
    return [dx + pdx, dy + pdy];
}

let currentTarget = null;
let lastState = null;
let imageDataBuffer = null; // reused across frames to avoid reallocating

// ─── Base occupancy layer via pixel buffer — cheap at any grid size ──────
function drawBaseLayer(robot_map) {
    if (!imageDataBuffer) {
        imageDataBuffer = ctx.createImageData(canvas.width, canvas.height);
    }
    const data = imageDataBuffer.data;
    const cell = CELL_SIZE;

    for (let x = 0; x < GRID_SIZE; x++) {
        for (let y = 0; y < GRID_SIZE; y++) {
            const prob = sigmoid(robot_map[x][y]);
            const brightness = Math.round((1 - prob) * 255);

            const px0 = Math.round(x * cell);
            // Flip y axis to match backend origin='lower'
            const py0 = Math.round((GRID_SIZE - 1 - y) * cell);
            const px1 = Math.round((x + 1) * cell);
            const py1 = Math.round((GRID_SIZE - 1 - y + 1) * cell);

            for (let py = py0; py < py1; py++) {
                let rowOffset = (py * canvas.width + px0) * 4;
                for (let px = px0; px < px1; px++) {
                    data[rowOffset]     = brightness;
                    data[rowOffset + 1] = brightness;
                    data[rowOffset + 2] = brightness;
                    data[rowOffset + 3] = 255;
                    rowOffset += 4;
                }
            }
        }
    }
    ctx.putImageData(imageDataBuffer, 0, 0);
}

// ─── Overlays: path/frontiers/target/robot — few cells, cheap vector draws ──
function drawOverlays(state) {
    const { robot_position, robot_direction, path } = state;
    const [rx, ry] = robot_position;
    const pathSet = new Set(path.map(([x, y]) => `${x},${y}`));

    const cellToScreen = (x, y) => [x * CELL_SIZE, (GRID_SIZE - 1 - y) * CELL_SIZE];

    // Path
    ctx.fillStyle = "#fdd835";
    for (const [x, y] of path) {
        const [px, py] = cellToScreen(x, y);
        ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);
    }

    // Frontiers
    if (state.frontiers) {
        ctx.fillStyle = state.exploring ? "#ff9800" : "#fdd835";
        for (const cluster of state.frontiers) {
            for (const [fx, fy] of cluster) {
                if (pathSet.has(`${fx},${fy}`)) continue;
                const [px, py] = cellToScreen(fx, fy);
                ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);
            }
        }
    }

    // Target
    if (currentTarget) {
        const [px, py] = cellToScreen(currentTarget[0], currentTarget[1]);
        ctx.fillStyle = "#42a5f5";
        ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);
    }

    // Robot + direction wedge
    const [rpx, rpy] = cellToScreen(rx, ry);
    ctx.fillStyle = "#e53935";
    ctx.fillRect(rpx, rpy, CELL_SIZE, CELL_SIZE);
    drawDirectionWedge(rpx, rpy, robot_direction);
}

function drawDirectionWedge(px, py, direction) {
    const [dx, dy] = DIR_SCREEN_VECTOR[direction] || [1, 0];
    const cx = px + CELL_SIZE / 2;
    const cy = py + CELL_SIZE / 2;
    const r = CELL_SIZE * 0.45;

    // Perpendicular vector for the wedge's base corners
    const perpX = -dy, perpY = dx;

    const tipX = cx + dx * r;
    const tipY = cy + dy * r;
    const baseLX = cx - dx * r * 0.5 + perpX * r * 0.6;
    const baseLY = cy - dy * r * 0.5 + perpY * r * 0.6;
    const baseRX = cx - dx * r * 0.5 - perpX * r * 0.6;
    const baseRY = cy - dy * r * 0.5 - perpY * r * 0.6;

    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(baseLX, baseLY);
    ctx.lineTo(baseRX, baseRY);
    ctx.closePath();
    ctx.fillStyle = "#ffffff";
    ctx.fill();
}

// Draws each sensor's current reading as a ray from the robot's cell, so the
// live sonar view can be compared directly against the accumulated robot_map
// underneath it. Dashed while sensor_connected is false (no real reading has
// arrived yet — the value shown is just the un-updated default).
function drawSensorRays(state) {
    const { robot_position, robot_direction, sensor_connected } = state;
    const [rx, ry] = robot_position;
    const cx = (rx + 0.5) * CELL_SIZE;
    const cy = (GRID_SIZE - 1 - ry + 0.5) * CELL_SIZE;

    const rays = [
        { side: null,    dist: state.sensor_distance_cm,       color: SENSOR_RAY_COLOR.front },
        { side: "left",  dist: state.sensor_distance_left_cm,  color: SENSOR_RAY_COLOR.left },
        { side: "right", dist: state.sensor_distance_right_cm, color: SENSOR_RAY_COLOR.right },
    ];

    ctx.lineWidth = 2;
    ctx.setLineDash(sensor_connected ? [] : [4, 4]);

    for (const ray of rays) {
        if (ray.dist == null) continue;
        const [dx, dy] = sensorRayVector(robot_direction, ray.side);
        const mag = Math.hypot(dx, dy) || 1;
        const lengthPx = (ray.dist / CELL_SIZE_CM) * CELL_SIZE;
        const ex = cx + (dx / mag) * lengthPx;
        const ey = cy + (dy / mag) * lengthPx;

        ctx.strokeStyle = ray.color;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(ex, ey);
        ctx.stroke();

        ctx.fillStyle = ray.color;
        ctx.beginPath();
        ctx.arc(ex, ey, 4, 0, Math.PI * 2);
        ctx.fill();
    }

    ctx.setLineDash([]);
}

function drawGrid(state) {
    drawBaseLayer(state.robot_map);
    drawOverlays(state);
    drawSensorRays(state);
}

// ─── Sigmoid ─────────────────────────────────────────────────
function sigmoid(x) {
    return 1 / (1 + Math.exp(-x));
}

// ─── Update Status Bar ───────────────────────────────────────
function updateStatus(state) {
    const dot = document.getElementById("status-dot");
    const statusText = document.getElementById("status-text");
    const posX = document.getElementById("pos-x");
    const posY = document.getElementById("pos-y");
    const goalText = document.getElementById("goal-text");

    posX.textContent = state.robot_position[0];
    posY.textContent = state.robot_position[1];

    document.getElementById("sonar-f").textContent = state.sensor_distance_cm.toFixed(0);
    document.getElementById("sonar-l").textContent = state.sensor_distance_left_cm.toFixed(0);
    document.getElementById("sonar-r").textContent = state.sensor_distance_right_cm.toFixed(0);

    if (state.is_moving) {
        dot.className = "moving";
        statusText.textContent = "Navigating";
    } else {
        dot.className = "";
        statusText.textContent = "Idle";
    }

    if (currentTarget) {
        goalText.textContent = `(${currentTarget[0]}, ${currentTarget[1]})`;
    } else {
        goalText.textContent = "None";
    }
}

// ─── Show Message ─────────────────────────────────────────────
function showMessage(msg, color = "#ff5252") {
    const el = document.getElementById("message");
    el.style.color = color;
    el.textContent = msg;
    setTimeout(() => el.textContent = "", 3000);
}

// ─── Fetch Map and Redraw ────────────────────────────────────
async function fetchAndDraw() {
    try {
        const res = await fetch(`${API}/map`);
        const state = await res.json();
        lastState = state;
        drawGrid(state);
        updateStatus(state);
    } catch (err) {
        showMessage("Cannot connect to server");
    }
}

// ─── Handle Click ─────────────────────────────────────────────
canvas.addEventListener("click", async (e) => {
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    // Convert pixel to grid coordinates
    const gridX = Math.floor(mouseX / CELL_SIZE);
    // Flip y to match backend
    const gridY = GRID_SIZE - 1 - Math.floor(mouseY / CELL_SIZE);

    // Boundary check
    if (gridX < 0 || gridX >= GRID_SIZE || gridY < 0 || gridY >= GRID_SIZE) return;

    // Don't navigate if already moving
    if (lastState && lastState.is_moving) {
        showMessage("Robot is moving, please wait");
        return;
    }

    currentTarget = [gridX, gridY];

    try {
        const res = await fetch(`${API}/navigate`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ x: gridX, y: gridY })
        });

        const data = await res.json();

        if (data.status === "error") {
            showMessage(data.message);
            currentTarget = null;
        } else {
            showMessage(`Navigating to (${gridX}, ${gridY})`, "#4caf50");
        }

    } catch (err) {
        showMessage("Navigation request failed");
    }
});

// ─── Manual Control ────────────────────────────────────────────
// No client-side is_moving guard here: /manual/move is designed to
// interrupt whatever's currently running (goto, explore, or a previous
// manual move) instead of rejecting — a guard based on `lastState` (which
// only refreshes every 300ms) would block legitimate follow-up commands
// (e.g. stop then left) while the poll is still stale.
async function manualMove(cmd) {
    try {
        const res = await fetch(`${API}/manual/move`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ cmd })
        });
        const data = await res.json();
        if (data.status === "error") {
            showMessage(data.message);
        } else {
            currentTarget = null; // manual driving cancels any goto target marker
            showMessage(`Manual: ${cmd}`, "#4caf50");
        }
    } catch (err) {
        showMessage("Manual move request failed");
    }
}

// ─── Speed Control ───────────────────────────────────────────────
// Independent left/right sliders rather than one speed + a fixed trim:
// measured drift direction wasn't consistent across test runs, so a single
// fixed-direction correction would assume a bias that doesn't always hold —
// letting the user dial in whichever side needs it that session instead.
const speedLeftSlider = document.getElementById("speed-left");
const speedRightSlider = document.getElementById("speed-right");
const speedLeftValue = document.getElementById("speed-left-value");
const speedRightValue = document.getElementById("speed-right-value");

async function postSpeed() {
    // Physical left/right are wired backwards from what the server's
    // left_pct/right_pct (ENA/ENB) assume — crossed here rather than in
    // firmware/server so the on-screen "Left"/"Right" labels stay true to
    // the actual chassis side; the API fields are just swapped in transit.
    const left_pct = parseInt(speedRightSlider.value, 10);
    const right_pct = parseInt(speedLeftSlider.value, 10);
    try {
        const res = await fetch(`${API}/speed`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ left_pct, right_pct })
        });
        const data = await res.json();
        speedRightSlider.value = data.left_pct;   // reflect server-side clamping (crossed back)
        speedLeftSlider.value = data.right_pct;
        speedRightValue.textContent = `${data.left_pct}%`;
        speedLeftValue.textContent = `${data.right_pct}%`;
        showMessage(`Speed set: L${data.right_pct}% / R${data.left_pct}%`, "#4caf50");
    } catch (err) {
        showMessage("Speed update failed");
    }
}

speedLeftSlider.addEventListener("change", postSpeed);
speedRightSlider.addEventListener("change", postSpeed);

speedLeftSlider.addEventListener("input", () => {
    speedLeftValue.textContent = `${speedLeftSlider.value}%`; // live label while dragging
});
speedRightSlider.addEventListener("input", () => {
    speedRightValue.textContent = `${speedRightSlider.value}%`;
});

document.addEventListener("keydown", (e) => {
    const keyMap = { ArrowUp: "forward", ArrowDown: "reverse", ArrowLeft: "left", ArrowRight: "right", " ": "stop" };
    const cmd = keyMap[e.key];
    if (!cmd) return;
    e.preventDefault();
    manualMove(cmd);
});

// ─── Poll Every 300ms ─────────────────────────────────────────
setInterval(fetchAndDraw, 300);

// Initial draw
fetchAndDraw();