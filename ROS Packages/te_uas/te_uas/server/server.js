import express from 'express';
import cors from 'cors';
import { LowSync } from 'lowdb';
import { JSONFileSync } from 'lowdb/node';
import dotenv from 'dotenv';

dotenv.config();

const defaultData = { flights: [] };
const db = new LowSync(new JSONFileSync('./src/te_uas/logs/logs_db.json'), defaultData);

// Initialize Express app
const app = express();

// Set up CORS and JSON middlewares
app.use(cors());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Basic home route for the API
app.get("/", (_req, res) => {
  res.send("Flight Logs API.\nUse /flights and /logs endpoints.");
});

// Get all flights
app.get("/flights", (_req, res) => {
  db.read();
  res.status(200).json(db.data.flights);
});

// Get current active flight, filtering out dismissed logs for the user
app.get("/flights/active", (req, res) => {
  db.read();
  const activeFlight = db.data.flights.find(flight => flight.active === true);
  if (activeFlight) {
    res.status(200).json(activeFlight);
  } else {
    res.status(404).json({ message: "No active flight found" });
  }
});


// Add log to current active flight
app.post("/logs", (req, res) => {
  const { level, description, name } = req.body;
  const timestamp = new Date().toLocaleString();
  db.read();
  const activeFlight = db.data.flights.find(flight => flight.active === true);
  if (activeFlight) {
    activeFlight.logs.push({ logId: generateUniqueId(new Set(db.data.flights.map(flight => flight.id))), level, description, timestamp, dismissed: [name] });
    db.write();
    res.status(200).json(activeFlight);
  } else {
    res.status(404).json({ message: "No active flight found to add log" });
  }
});

// New endpoint to end a flight and log the action
app.post("/flight/end", (req, res) => {
  const { user } = req.body;
  const timestamp = new Date().toLocaleString();
  db.read();  
  const activeFlight = db.data.flights.find(flight => flight.active === true);
  if (activeFlight) {
    // Log the mode change action
    const log = {
      level: 'INFO',
      description: `[${user.role}] ${user.name} changed mode to AUTO.LAND`,
      timestamp,
      dismissed: []
    };
    activeFlight.logs.push(log);

    // Change flight status to inactive and set landedBy
    activeFlight.active = false;
    activeFlight.info.landedBy = user.name;
    activeFlight.info.landedAt = timestamp;

    db.write();
    res.status(200).json({ message: "Flight mode changed and logged successfully", flight: activeFlight });
  } else {
    res.status(404).json({ message: "No active flight found to change mode" });
  }
});

// Create a new flight
app.post("/flight/start", (req, res) => {
  const { tookoffBy } = req.body.info;
  const tookoffAt = new Date().toLocaleString();
  db.read();
  // Deactivate any active flights
  db.data.flights.forEach(flight => flight.active = false);
  const newFlight = {
    id: generateUniqueId(new Set(db.data.flights.map(flight => flight.id))),
    active: true,
    info: { tookoffBy, tookoffAt, landedBy: "", landedAt: "" },
    logs: []
  };
  db.data.flights.push(newFlight);
  db.write();
  res.status(201).json(newFlight);
});

// Dismiss a log for a specific user
app.post("/logs/dismiss", (req, res) => {
  const { name, logId } = req.body;
  db.read();
  const activeFlight = db.data.flights.find(flight => flight.active === true);
  if (activeFlight) {
    const log = activeFlight.logs.find(log => log.logId === logId);
    if (log) {
      if (!log.dismissed.includes(name)) {
        log.dismissed.push(name);
        db.write();
        res.status(200).json({ updatedLogs: activeFlight.logs, message: "Log dismissed successfully" });
      } else {
        res.status(400).json({ message: "Log already dismissed by this user" });
      }
    } else {
      res.status(404).json({ message: "Log not found" });
    }
  } else {
    res.status(404).json({ message: "No active flight found" });
  }
});

function generateUniqueId(existingIds) {
  let id;
  do {
    id = Math.floor(Math.random() * 99999);
  } while (existingIds.has(id));
  existingIds.add(id);
  return id;
}

app.listen(3081, () => {
  console.log("Flight Logs Server running on port 3081");
});
