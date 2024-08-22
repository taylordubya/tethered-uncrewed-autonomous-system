/* Styling */
import './App.css';

/* Default Imports */
import { useEffect, useState } from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';

/* Imported Packages */
import axios from 'axios';

/* Components */
import Navbar from './Navbar/Navbar';
import Notifications from './Notifications/Notifications';

import Login from './Login/Login';
import Home from './Home';
import Controls from './Controls/Controls';
import Logs from './LogsPage/Logs';
import Settings from './Settings/Settings';

/* Performs checks to see if the user is authenticated or not */
function checkAuth(serverIp, setLoggedIn) {
  /* Get the user from local storage */
  const user = JSON.parse(localStorage.getItem('user'));

  /* Check 1: if the user exists and has a token */
  if (!user || !user.token) {
    setLoggedIn(false);
    return;
  }

  /* Check 2: the user's token is valid */
  const verifyUser = async () => {
    try {
      const response = await axios.post(
        `http://${serverIp}:3080/auth/verify`,
        {},
        {
          headers: {
            'jwt-token': user.token,
          },
        },
      );
      if (response.data.message === 'success') {
        setLoggedIn(true);
      } else {
        setLoggedIn(false);
      }
    } catch (error) {
      /* Cleanup for possible issue with the stored user */
      localStorage.removeItem('user');
      setLoggedIn(false);
    }
  };
  verifyUser();
}

async function populateLogs(serverIp, setLogs) {
  const user = JSON.parse(localStorage.getItem('user'));
  if (user) {
    try {
      const response = await axios.get(
        `http://${serverIp}:3081/flights?active=true`,
        {
          headers: { 'jwt-token': user.token },
        },
      );

      if (response.data.logs) {
        setLogs(response.data.logs);
      }
    } catch (error) {
      // console.error('Error fetching active logs:', error);
    }
  }
}

async function logConnected(serverIp) {
  const user = JSON.parse(localStorage.getItem('user'));
  if (user) {
    const log = {
      level: 'INFO',
      description: `[${user.role}] ${user.name} has connected`,
      name: user.name,
    };

    try {
      await axios.post(`http://${serverIp}:3081/logs`, log, {
        headers: { 'jwt-token': user.token },
      });
    } catch (error) {
      // console.error('Error logging user connection:', error);
    }
  }
}

export default function App() {
  /* App States */
  const [loggedIn, setLoggedIn] = useState(false);

  /* Configurations */
  const ip = '10.42.0.1';
  const [serverIp, setServerIp] = useState('localhost');
  const [logs, setLogs] = useState([]); // List of occurred logs, auto-populates from server

  /* Connections */
  const [rosConnected, setRosConnected] = useState(false);
  const [serverConnected, setServerConnected] = useState(false);

  /* Misc */
  const [armed, setArmed] = useState(false);
  const [notificationVisible, setNotificationVisible] = useState(true); // To hide notification window on certain pages

  /* Check server connection status */
  useEffect(() => {
    const checkConnection = async () => {
      try {
        await axios.get(`http://${serverIp}:3080/`);
        setServerConnected(true);
      } catch (error) {
        setServerConnected(false);
      }
    };
    checkConnection();
  }, [serverIp]);

  /* Check user authentication status */
  useEffect(() => {
    checkAuth(serverIp, setLoggedIn);
  }, [serverIp]);

  /* Populate logs */
  useEffect(() => {
    if (serverConnected) {
      populateLogs(serverIp, setLogs);
    }
  }, [loggedIn, rosConnected, serverConnected, serverIp]);

  /* Send LOG when connected */
  useEffect(() => {
    if (loggedIn && rosConnected && armed) {
      logConnected(serverIp);
    }
  }, [loggedIn, rosConnected, armed, serverIp]);

  return (
    <div className="App">
      <Router>
        {loggedIn && <Navbar ip={ip} armed={armed} setLoggedIn={setLoggedIn} />}
        <Routes>
          <Route
            path="*"
            element={
              <Login
                serverConnected={serverConnected}
                serverIp={serverIp}
                setServerIp={setServerIp}
                loggedIn={loggedIn}
                setLoggedIn={setLoggedIn}
              />
            }
          />
          <Route
            path="/home"
            element={
              <Home
                rosConnected={rosConnected}
                setRosConnected={setRosConnected}
                ip={ip}
                serverIp={serverIp}
                loggedIn={loggedIn}
                setLoggedIn={setLoggedIn}
                armed={armed}
                setArmed={setArmed}
                setLogs={setLogs}
              />
            }
          />
          <Route
            path="/controls"
            element={<Controls ip={ip} loggedIn={loggedIn} />}
          />
          <Route
            path="/logs"
            element={
              <Logs
                setNotificationVisible={setNotificationVisible}
                serverIp={serverIp}
                logs={logs}
              />
            }
          />
          <Route
            path="/settings"
            element={
              <Settings
                serverIp={serverIp}
                loggedIn={loggedIn}
                setLoggedIn={setLoggedIn}
              />
            }
          />
        </Routes>
        {loggedIn && (
          <Notifications
            notificationVisible={notificationVisible}
            serverIp={serverIp}
            logs={logs}
            setLogs={setLogs}
          />
        )}
      </Router>
    </div>
  );
}