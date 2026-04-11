/**
 * Scanner Debug Mode — Motor control panel logic
 * Handles profile loading, connection, motor commands, and position polling.
 */

// ── API Helper ───────────────────────────────────────────────────────────────

const API = {
  async get(url) {
    const res = await fetch(url);
    if (!res.ok) {
      const text = await res.text();
      throw new Error(text);
    }
    return res.json();
  },
  async post(url, data = {}) {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data),
    });
    if (!res.ok) {
      const text = await res.text();
      throw new Error(text);
    }
    return res.json();
  },
};

// ── Role → Color Mapping ─────────────────────────────────────────────────────

const ROLE_COLORS = {
  specimen_rotation: "#00ff88",
  turntable: "#00ff88",
  focus: "#ff6644",
  focus_rail: "#ff6644",
  camera_tilt: "#4488ff",
  camera_orbit: "#ff8800",
  positioning: "#dddd44",
  bed_positioning: "#dddd44",
};

function getAxisColor(role) {
  return ROLE_COLORS[role] || "#aaaacc";
}

// ── State ────────────────────────────────────────────────────────────────────

let currentAxes = [];
let pollingInterval = null;

// ── Init ─────────────────────────────────────────────────────────────────────

document.addEventListener("DOMContentLoaded", () => {
  document.getElementById("connect-btn").addEventListener("click", doConnect);
  document
    .getElementById("disconnect-btn")
    .addEventListener("click", doDisconnect);
  document.getElementById("power-btn").addEventListener("click", togglePower);
  document.getElementById("stop-btn").addEventListener("click", emergencyStop);
  document.getElementById("home-all-btn").addEventListener("click", homeAll);

  const select = document.getElementById("profile-select");
  select.addEventListener("change", previewProfile);
  if (select.value) previewProfile();
});

// ── Profile Preview ──────────────────────────────────────────────────────────

async function previewProfile() {
  const name = document.getElementById("profile-select").value;
  if (!name) return;

  try {
    const profile = await API.get(`/api/profiles/${name}`);
    currentAxes = profile.axes;

    // Update 3D viz even before connecting
    if (window.scannerViz) {
      window.scannerViz.setProfile(profile.axes);
    }

    // Show preview cards (disabled)
    if (!document.getElementById("disconnect-btn").disabled === false) {
      renderMotorCards(profile.axes, {}, false);
    }
  } catch (e) {
    console.error("Failed to load profile:", e);
  }
}

// ── Connect / Disconnect ─────────────────────────────────────────────────────

async function doConnect() {
  const profile = document.getElementById("profile-select").value;
  const mock = document.getElementById("mock-toggle").checked;

  setStatus("connecting");

  try {
    await API.post("/api/connect", { profile, mock });
    setStatus("connected");
    setConnectedUI(true);

    const status = await API.get("/api/status");
    currentAxes = status.axes;
    renderMotorCards(status.axes, status.positions, true);

    if (window.scannerViz) {
      window.scannerViz.setProfile(status.axes);
      window.scannerViz.updatePositions(status.positions);
    }

    startPolling();
  } catch (e) {
    setStatus("disconnected");
    console.error("Connect failed:", e);
  }
}

async function doDisconnect() {
  stopPolling();
  try {
    await API.post("/api/disconnect");
  } catch (e) {
    console.error("Disconnect error:", e);
  }
  setStatus("disconnected");
  setConnectedUI(false);
  document.getElementById("motor-cards").innerHTML =
    '<p class="placeholder">Select a profile and connect to see motors.</p>';
}

function setStatus(status) {
  const el = document.getElementById("status-indicator");
  el.className = `status ${status}`;
  el.textContent = status.charAt(0).toUpperCase() + status.slice(1);
}

function setConnectedUI(connected) {
  document.getElementById("connect-btn").disabled = connected;
  document.getElementById("profile-select").disabled = connected;
  document.getElementById("mock-toggle").disabled = connected;
  document.getElementById("disconnect-btn").disabled = !connected;
  document.getElementById("power-btn").disabled = !connected;
  document.getElementById("stop-btn").disabled = !connected;
  document.getElementById("home-all-btn").disabled = !connected;
}

// ── Motor Cards ──────────────────────────────────────────────────────────────

function renderMotorCards(axes, positions, interactive) {
  const container = document.getElementById("motor-cards");
  container.innerHTML = "";

  for (const axis of axes) {
    const pos = positions[axis.name] ?? 0;
    const color = getAxisColor(axis.role);
    const range = axis.max_limit - axis.min_limit;
    const pct = range > 0 ? ((pos - axis.min_limit) / range) * 100 : 0;

    const isRotary = axis.axis_type === "rotary";
    const decimals = isRotary ? 1 : 2;
    const steps = isRotary ? [90, 10, 1] : [10, 1, 0.1];
    const dis = interactive ? "" : "disabled";

    const card = document.createElement("div");
    card.className = "motor-card";
    card.id = `card-${axis.name}`;
    card.style.borderLeftColor = color;

    card.innerHTML = `
            <div class="card-header">
                <span class="axis-name" style="color:${color}">${axis.name}</span>
                <span class="axis-label">${axis.label} · ${axis.axis_type}</span>
            </div>
            <div class="position" id="pos-${axis.name}">${pos.toFixed(decimals)} ${axis.units}</div>
            <div class="limits">${axis.min_limit}${axis.units} → ${axis.max_limit}${axis.units}</div>
            <div class="position-bar">
                <div class="fill" id="bar-${axis.name}"
                     style="width:${clamp(pct)}%; background:${color}"></div>
            </div>
            <div class="jog-buttons">
                ${steps
                  .map(
                    (s) =>
                      `<button class="btn jog-neg" onclick="jog('${axis.name}',${-s})" ${dis}>◄${s}</button>`,
                  )
                  .join("")}
                <div class="jog-separator"></div>
                ${steps
                  .slice()
                  .reverse()
                  .map(
                    (s) =>
                      `<button class="btn jog-pos" onclick="jog('${axis.name}',${s})" ${dis}>►${s}</button>`,
                  )
                  .join("")}
            </div>
            <div class="move-row">
                <input type="number" id="goto-${axis.name}" placeholder="Go to…"
                       step="${isRotary ? 1 : 0.1}" ${dis}>
                <button class="btn btn-primary" onclick="moveTo('${axis.name}')" ${dis}>Go</button>
            </div>
            <div class="card-actions">
                <button class="btn" onclick="homeAxis('${axis.name}')" ${dis}>⌂ Home</button>
                <button class="btn" onclick="zeroAxis('${axis.name}')" ${dis}>⓪ Zero</button>
            </div>
        `;

    container.appendChild(card);
  }
}

function updatePositionDisplay(name, pos) {
  const axis = currentAxes.find((a) => a.name === name);
  if (!axis) return;

  const decimals = axis.axis_type === "rotary" ? 1 : 2;
  const posEl = document.getElementById(`pos-${name}`);
  if (posEl) posEl.textContent = `${pos.toFixed(decimals)} ${axis.units}`;

  const range = axis.max_limit - axis.min_limit;
  const pct = range > 0 ? ((pos - axis.min_limit) / range) * 100 : 0;
  const barEl = document.getElementById(`bar-${name}`);
  if (barEl) barEl.style.width = `${clamp(pct)}%`;
}

function clamp(v) {
  return Math.max(0, Math.min(100, v));
}

// ── Motor Commands (global for onclick) ──────────────────────────────────────

window.jog = async function (axisName, distance) {
  try {
    const result = await API.post(`/api/axes/${axisName}/jog`, { distance });
    if (result.success) {
      updatePositionDisplay(axisName, result.position);
      if (window.scannerViz) {
        window.scannerViz.updateAxisPosition(axisName, result.position);
      }
    } else {
      console.warn(`Jog ${axisName} failed: ${result.error}`);
    }
  } catch (e) {
    console.error("Jog failed:", e);
  }
};

window.moveTo = async function (axisName) {
  const input = document.getElementById(`goto-${axisName}`);
  const position = parseFloat(input.value);
  if (isNaN(position)) return;

  try {
    const result = await API.post(`/api/axes/${axisName}/move-to`, {
      position,
    });
    if (result.success) {
      updatePositionDisplay(axisName, result.position);
      if (window.scannerViz) {
        window.scannerViz.updateAxisPosition(axisName, result.position);
      }
    }
  } catch (e) {
    console.error("Move failed:", e);
  }
};

window.homeAxis = async function (axisName) {
  try {
    const result = await API.post(`/api/axes/${axisName}/home`);
    if (result.success) {
      updatePositionDisplay(axisName, result.position);
      if (window.scannerViz) {
        window.scannerViz.updateAxisPosition(axisName, result.position);
      }
    }
  } catch (e) {
    console.error("Home failed:", e);
  }
};

window.zeroAxis = async function (axisName) {
  try {
    const result = await API.post(`/api/axes/${axisName}/zero`);
    if (result.success) {
      updatePositionDisplay(axisName, result.position);
      if (window.scannerViz) {
        window.scannerViz.updateAxisPosition(axisName, result.position);
      }
    }
  } catch (e) {
    console.error("Zero failed:", e);
  }
};

async function togglePower() {
  const btn = document.getElementById("power-btn");
  const turningOn = btn.textContent.includes("ON");

  try {
    const result = await API.post("/api/power", { on: turningOn });
    btn.textContent = result.power ? "Power OFF" : "Power ON";
    btn.className = result.power ? "btn btn-danger" : "btn btn-warning";
  } catch (e) {
    console.error("Power toggle failed:", e);
  }
}

async function emergencyStop() {
  try {
    await API.post("/api/stop");
  } catch (e) {
    console.error("Stop failed:", e);
  }
}

async function homeAll() {
  for (const axis of currentAxes) {
    await window.homeAxis(axis.name);
  }
}

// ── Position Polling ─────────────────────────────────────────────────────────

function startPolling() {
  stopPolling();
  pollingInterval = setInterval(async () => {
    try {
      const status = await API.get("/api/status");
      for (const [name, pos] of Object.entries(status.positions)) {
        updatePositionDisplay(name, pos);
      }
      if (window.scannerViz) {
        window.scannerViz.updatePositions(status.positions);
      }
    } catch (_) {
      // Ignore polling errors
    }
  }, 500);
}

function stopPolling() {
  if (pollingInterval) {
    clearInterval(pollingInterval);
    pollingInterval = null;
  }
}
