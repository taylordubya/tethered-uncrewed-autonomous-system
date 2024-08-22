import express from 'express';
import bcrypt from 'bcryptjs';
import cors from 'cors';
import jwt from 'jsonwebtoken';
import { LowSync } from 'lowdb';
import { JSONFileSync } from 'lowdb/node';
import dotenv from 'dotenv';

/* Configuration */
dotenv.config(); // Load environment variables from .env file
const saltRounds = 10; // Number of salt rounds for bcrypt

const defaultData = { users: [] }; // Default data structure for the database
const databasePath = process.env.TE_UAS_AUTH_PATH; // Path to the database file
const db = new LowSync(new JSONFileSync(databasePath), defaultData); // Initialize LowDB with the JSON file adapter and default data

// Initialize Express app
const app = express();

// Define a JWT secret key from environment variables
const jwtSecretKey = process.env.JWT_SECRET_KEY;

// Set up CORS and JSON middlewares
app.use(cors());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Basic home route for the API
app.get("/", (_req, res) => {
  res.send(
    "Auth API.\nPlease use POST /auth & POST /verify for authentication"
  );
});

// The auth endpoint that logs a user in based on an existing record
app.post("/auth", (req, res) => {
  console.log("Authentication Requested");
  db.read(); // Read the database
  const pin = req.body.fullPin; // Entered pin
  const users = db.data.users; // Every registered user in the database

  // Find the user with a matching pin
  const user = users.find(user => bcrypt.compareSync(pin, user.pin));
  if (user) {
    // PIN CORRECT
    let loginData = {
      name: user.name,
      role: user.role
    };
    // Generate a JWT token
    const token = jwt.sign(loginData, jwtSecretKey, { expiresIn: '1h' });
    res.status(200).json({ message: "success", name: user.name, role: user.role, token: token });
  } else {
    res.status(401).json({ message: "Invalid pin" }); // Invalid pin
  }
});

// The verify endpoint that checks if a given JWT token is valid
app.post("/auth/verify", (req, res) => {
  const tokenHeaderKey = "jwt-token"; // JWT token header key
  const authToken = req.headers[tokenHeaderKey]; // JWT token from headers
  try {
    // Verify the token
    const verified = jwt.verify(authToken, jwtSecretKey);
    if (verified) {
      return res.status(200).json({ status: "logged in", message: "success" });
    } else {
      // Access Denied
      return res.status(401).json({ status: "invalid auth", message: "error" });
    }
  } catch (error) {
    // Access Denied
    return res.status(401).json({ status: "invalid auth", message: "error" });
  }
});

// Combined endpoint for user management
app.post("/users", (req, res) => {
  const tokenHeaderKey = "jwt-token"; // JWT token header key
  const authToken = req.headers[tokenHeaderKey]; // JWT token from headers
  const { action, user } = req.body; // Extract action and user data from request body

  try {
    // Verify the token
    const verified = jwt.verify(authToken, jwtSecretKey);

    if (verified && verified.role === "Administrator") {
      db.read(); // Read the database
      const existingIds = new Set(db.data.users.map(u => u.id)); // Get existing user IDs

      if (action === "add") {
        // Create new user object
        const hashedPin = bcrypt.hashSync(user.pin, saltRounds);
        const newUser = {
          pin: hashedPin,
          id: generateUniqueId(existingIds), // Generate unique ID
          name: user.name,
          role: user.role
        };

        db.data.users.push(newUser); // Add new user to the database
        db.write(); // Write changes to the database
        res.status(200).json({ users: db.data.users, message: "User created successfully" });
      } else if (action === "remove") {
        // Filter out the user to be removed
        const updatedUsers = db.data.users.filter(u => u.id !== user.id);
        db.data.users = updatedUsers; // Update users array
        db.write(); // Write changes to the database
        res.status(200).json({ users: db.data.users, message: "User deleted successfully" });
      } else if (action === "edit") {
        // Find the user to make edits to, throw error if user doesn't exist
        const userIndex = db.data.users.findIndex(u => u.id === user.id);
        if (userIndex === -1) {
          return res.status(404).send({ message: "User not found." });
        }

        // Set the new pin
        const hashedPin = bcrypt.hashSync(user.pin, saltRounds);
        db.data.users[userIndex].pin = hashedPin;
        // Set the new role
        db.data.users[userIndex].role = user.role;
        db.write(); // Write changes to the database
        res.status(200).json({ users: db.data.users, message: "User edited successfully" });
      } else {
        res.status(400).json({ message: "Invalid action specified" }); // Invalid action
      }
    } else {
      // Access Denied
      res.status(401).json({ status: "invalid auth", message: "error" });
    }
  } catch (error) {
    // Access Denied
    res.status(401).json({ status: "invalid auth", message: "error" });
  }
});

// Endpoint to get all users
app.get("/users", (req, res) => {
  const tokenHeaderKey = "jwt-token"; // JWT token header key
  const authToken = req.headers[tokenHeaderKey]; // JWT token from headers
  try {
    // Verify the token
    const verified = jwt.verify(authToken, jwtSecretKey);
    // Check if the user has admin access
    if (verified && verified.role === "Administrator") {
      db.read(); // Read the database
      const users = db.data.users; // Get all users
      res.status(200).json({ users: users });
    } else {
      // Access Denied
      res.status(401).json({ status: "invalid auth", message: "error" });
    }
  } catch (error) {
    // Access Denied
    res.status(401).json({ status: "invalid auth", message: "error" });
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
app.listen(3080, () => {
  console.log("Auth Server running on port 3080");
});
