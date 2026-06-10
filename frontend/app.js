const API = "http://127.0.0.1:8000";
const canvas = document.getElementById("grid");
const ctx = canvas.getContext("2d");

const GRID_SIZE = 10;
const CELL_SIZE = canvas.width / GRID_SIZE;

let currentTarget = null;
let lastState = null;

// ─── Draw Grid ───────────────────────────────────────────────
function drawGrid(state) {
    const { robot_map, robot_position, path, is_moving } = state;
    const [rx, ry] = robot_position;

    // Build path set for fast lookup
    const pathSet = new Set(path.map(([x, y]) => `${x},${y}`));

    for (let x = 0; x < GRID_SIZE; x++) {
        for (let y = 0; y < GRID_SIZE; y++) {

            const prob = sigmoid(robot_map[x][y]);
            const px = x * CELL_SIZE;
            // Flip y axis to match backend origin='lower'
            const py = (GRID_SIZE - 1 - y) * CELL_SIZE;

            // Determine cell color
            if (x === rx && y === ry) {
                ctx.fillStyle = "#e53935"; // robot — red

            } else if (currentTarget && x === currentTarget[0] && y === currentTarget[1]) {
                ctx.fillStyle = "#42a5f5"; // target — blue

            } else if (pathSet.has(`${x},${y}`)) {
                ctx.fillStyle = "#fdd835"; // path — yellow
                
            // In drawGrid, add this before the grayscale else block:
            } else if (state.frontiers && state.frontiers.some(cluster =>
                cluster.some(([fx, fy]) => fx === x && fy === y))) {
                ctx.fillStyle = state.exploring ? "#ff9800" : "#fdd835"; // orange if exploring

            } else {
                // Probability to grayscale
                // prob=0.5 → gray, prob=0 → white, prob=1 → black
                const brightness = Math.round((1 - prob) * 255);
                ctx.fillStyle = `rgb(${brightness},${brightness},${brightness})`;
            }

            ctx.fillRect(px, py, CELL_SIZE, CELL_SIZE);

            // Grid lines
            ctx.strokeStyle = "#333";
            ctx.lineWidth = 0.5;
            ctx.strokeRect(px, py, CELL_SIZE, CELL_SIZE);
        }
    }
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

// ─── Poll Every 300ms ─────────────────────────────────────────
setInterval(fetchAndDraw, 300);

// Initial draw
fetchAndDraw();