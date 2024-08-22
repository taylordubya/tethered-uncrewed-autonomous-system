import express from 'express';
import cors from 'cors';
import { LowSync } from 'lowdb';
import { JSONFileSync } from 'lowdb/node';
import dotenv from 'dotenv';

dotenv.config();

// Default data structure for the database
const defaultData = { flights: [] };

// Path to the database file
const databasePath = process.env.TE_UAS_LOG_PATH;

// Initialize LowDB with the JSON file adapter and default data
const db = new LowSync(new JSONFileSync(databasePath), defaultData);

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

// Get flights endpoint with optional query for active flight
app.get("/flights", (req, res) => {
  db.read(); // Read the database
  const { active } = req.query; // Get the 'active' query parameter

  if (active === 'true') {
    // If 'active' is true, find the active flight
    const activeFlight = db.data.flights.find(flight => flight.active === true);
    if (activeFlight) {
      res.status(200).json(activeFlight); // Respond with the active flight
    } else {
      res.status(404).json({ message: "No active flight found" }); // No active flight found
    }
  } else {
    res.status(200).json(db.data.flights); // Respond with all flights
  }
});

// Add log to current active flight
app.post("/logs", (req, res) => {
  const { level, description } = req.body; // Extract log details from request body
  const timestamp = new Date().toLocaleString(); // Get current timestamp
  db.read(); // Read the database
  const activeFlight = db.data.flights.find(flight => flight.active === true); // Find the active flight
  if (activeFlight) {
    // Add the new log to the active flight
    activeFlight.logs.push({
      logId: generateUniqueId(new Set(db.data.flights.map(flight => flight.id))), // Generate unique ID for the log
      level,
      description,
      timestamp,
      dismissed: []
    });
    db.write(); // Write changes to the database
    res.status(200).json(activeFlight); // Respond with the updated active flight
  } else {
    res.status(404).json({ message: "No active flight found to add log" }); // No active flight found
  }
});

// End a flight and log it has ended or start a new flight
app.post("/flight", (req, res) => {
  const { user, action } = req.body; // Extract user and action from request body
  const timestamp = new Date().toLocaleString(); // Get current timestamp
  db.read(); // Read the database
  const activeFlight = db.data.flights.find(flight => flight.active === true); // Find the active flight

  if (action === 'end') {
    if (activeFlight) {
      // Log the mode change action
      const log = {
        level: 'INFO',
        description: `[${user.role}] ${user.name} changed mode to AUTO.LAND`,
        timestamp,
        dismissed: []
      };
      activeFlight.logs.push(log); // Add log to active flight

      // Change flight status to inactive and set landedBy
      activeFlight.active = false;
      activeFlight.info.landedBy = user.name;
      activeFlight.info.landedAt = timestamp;

      db.write(); // Write changes to the database
      res.status(200).json({ message: "Flight mode changed and logged successfully", flight: activeFlight });
    } else {
      res.status(404).json({ message: "No active flight found to change mode" }); // No active flight found
    }
  } else if (action === 'start') {
    const { tookoffBy } = req.body.info; // Extract tookoffBy from request body
    const tookoffAt = new Date().toLocaleString(); // Get current timestamp
    // Deactivate any active flights
    db.data.flights.forEach(flight => flight.active = false);
    // Create a new flight
    const newFlight = {
      id: generateUniqueId(new Set(db.data.flights.map(flight => flight.id))), // Generate unique ID for the new flight
      active: true,
      info: { tookoffBy, tookoffAt, landedBy: "", landedAt: "" },
      logs: []
    };
    db.data.flights.push(newFlight); // Add new flight to the database
    db.write(); // Write changes to the database
    res.status(201).json(newFlight); // Respond with the new flight
  } else {
    res.status(400).json({ message: "Invalid action specified" }); // Invalid action
  }
});

// Dismiss a log or all logs for a specific user
app.post("/logs/dismiss", (req, res) => {
  const { name, logId } = req.body; // Extract name and logId from request body
  db.read(); // Read the database
  const activeFlight = db.data.flights.find(flight => flight.active === true); // Find the active flight

  if (activeFlight) {
    if (logId) {
      // If logId is provided, dismiss the specific log
      const log = activeFlight.logs.find(log => log.logId === logId);
      if (log) {
        if (!log.dismissed.includes(name)) {
          log.dismissed.push(name); // Add user to dismissed list for the log
          db.write(); // Write changes to the database
          res.status(200).json({ updatedLogs: activeFlight.logs, message: "Log dismissed successfully" });
        } else {
          res.status(400).json({ message: "Log already dismissed by this user" }); // Log already dismissed
        }
      } else {
        res.status(404).json({ message: "Log not found" }); // Log not found
      }
    } else {
      // If no logId is provided, dismiss all logs for the user
      activeFlight.logs.forEach(log => {
        if (!log.dismissed.includes(name)) {
          log.dismissed.push(name); // Add user to dismissed list for each log
        }
      });
      db.write(); // Write changes to the database
      res.status(200).json({ updatedLogs: activeFlight.logs, message: "All logs dismissed successfully for the user" });
    }
  } else {
    res.status(404).json({ message: "No active flight found" }); // No active flight found
  }
});

// Function to generate a unique ID
function generateUniqueId(existingIds) {
  let id;
  do {
    id = Math.floor(Math.random() * 99999);
  } while (existingIds.has(id));
  existingIds.add(id);
  return id;
}

// Start the server
app.listen(3081, () => {
  console.log("Flight Logs Server running on port 3081");
});
