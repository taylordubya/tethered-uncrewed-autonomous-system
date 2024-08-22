import express from 'express';
import bcrypt from 'bcryptjs';
import cors from 'cors';
import jwt from 'jsonwebtoken';
import { LowSync } from 'lowdb';
import { JSONFileSync } from 'lowdb/node';
import dotenv from 'dotenv';

dotenv.config();

const saltRounds = 10;

const defaultData = { users: [] };
const db = new LowSync(new JSONFileSync('./src/te_uas/te_uas/auth-server/database.json'), defaultData);

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

// The auth endpoint that creates a new user record or logs a user based on an existing record
app.post("/auth", (req, res) => {
  console.log("Authentication Requested");
  db.read();
  const pin = req.body.fullPin; // Entered pin
  const users = db.data.users; // Every registered user in the database

  const user = users.find(user => bcrypt.compareSync(pin, user.pin));
  if (user) {
    // PIN CORRECT
    let loginData = {
      name: user.name,
      role: user.role
    };
    const token = jwt.sign(loginData, jwtSecretKey, { expiresIn: '1h' });
    res.status(200).json({ message: "success", name: user.name, role: user.role, token: token });
  } else {
    res.status(401).json({ message: "Invalid pin" });
  }
});

// The verify endpoint that checks if a given JWT token is valid
app.post("/verify", (req, res) => {
  const tokenHeaderKey = "jwt-token";
  const authToken = req.headers[tokenHeaderKey];
  try {
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

/* ADMINISTRATOR ONLY ENDPOINTS BELOW */

app.get("/users", (req, res) => {
  const tokenHeaderKey = "jwt-token";
  const authToken = req.headers[tokenHeaderKey];
  try {
    const verified = jwt.verify(authToken, jwtSecretKey);
    if (verified && req.headers.role === "Administrator") {
      // Fetch users from the database
      db.read();
      const users = db.data.users;
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


function generateUniqueId(existingIds) {
  let id;
  do {
    id = Math.floor(Math.random() * 99999);
  } while (existingIds.has(id));
  existingIds.add(id);
  return id;
}

app.post("/add", (req, res) => {
  const tokenHeaderKey = "jwt-token";
  const authToken = req.headers[tokenHeaderKey];
  try {
    // Extra check to make sure user has admin access
    const verified = jwt.verify(authToken, jwtSecretKey);
    if (verified && req.body.user.auth === "Administrator") {
      db.read();
      const existingIds = new Set(db.data.users.map(user => user.id));
      const userData = req.body.user
      
      const newUser = {
        pin: userData.pin, 
        id: generateUniqueId(existingIds), 
        name: userData.name, 
        role: userData.role
      }
      
      console.log(db.data)
      db.data.users.push(newUser);
      db.write();
      res.status(200).json({ users: db.data.users, message: "User created successfully" });
    }
  } catch (error) {
    // Access Denied
  }
})

app.post("/remove", (req, res) => {
  const tokenHeaderKey = "jwt-token";
  const authToken = req.headers[tokenHeaderKey];
  try {
    // Extra check to make sure user has admin access
    const verified = jwt.verify(authToken, jwtSecretKey);
    if (verified && req.body.req.auth === "Administrator") {
      db.read();
      const updatedUsers = db.data.users.filter((user) => user.id !== req.body.req.id);
      db.data.users = updatedUsers;
      db.write();
      
      res.status(200).json({ users: db.data.users, message: "User deleted successfully" });
    }
  } catch (error) {

  }
})

app.post("/edit", (req, res) => {
  const tokenHeaderKey = "jwt-token";
  const authToken = req.headers[tokenHeaderKey];
  try {
    const verified = jwt.verify(authToken, jwtSecretKey);
    if (verified && req.body.user.auth === "Administrator") {
      const { userId, hashedPin, role } = req.body.user;
      /* Read the database */
      db.read();

      /* Find the user to make edits to, throw error if user doesn't exist */
      const userIndex = db.data.users.findIndex(user => user.id === userId);
      if (userIndex === -1) {
        return res.status(404).send({ message: "User not found." });
      }

      /* Set the new pin */
      db.data.users[userIndex].pin = hashedPin;
      db.write()

      res.status(200).json({ users: db.data.users, message: "User edited successfully" });
    }
  } catch (error) {
    res.status(500)
  }
})

app.listen(3080, () => {
  console.log("Auth Server running on port 3080");
});
