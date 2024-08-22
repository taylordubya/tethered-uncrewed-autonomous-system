/* eslint-disable promise/always-return */
/* Styling */
import './Login.css';

/* Default Imports */
import PropTypes from 'prop-types';
import { useEffect, useState, useCallback } from 'react';
import { useNavigate } from 'react-router-dom';

/* Imported Packages */
import axios from 'axios';
import logo from '../../../assets/MDR-logo.png';

/* Components */
import KeyPad from './Components/KeyPad';

/*  */
function checkAuth(serverIp, setLoggedIn, navigate) {
  const user = JSON.parse(localStorage.getItem('user'));
  if (!user || !user.token) {
    setLoggedIn(false);
    return; // Explicitly returning to exit the function early
  }

  axios
    .post(
      `http://${serverIp}:3080/auth/verify`,
      {},
      {
        headers: {
          'jwt-token': user.token,
        },
      },
    )
    .then((response) => {
      if (response.data.message === 'success') {
        setLoggedIn(true);
        navigate('/home');
      }
      setLoggedIn(false);
    })
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    .catch((error) => {
      setLoggedIn(false);
    });
}

function Login({
  serverConnected,
  serverIp,
  setServerIp,
  loggedIn,
  setLoggedIn,
}) {
  const [pin, setPin] = useState([]);
  const [pinError, setPinError] = useState();
  const [activeKey, setActiveKey] = useState(null);
  const [ipAddress, setIpAddress] = useState(serverIp);
  const MAX_INPUT_LENGTH = 4; // Length of the pin

  const navigate = useNavigate();

  /* Return a formatted version of a status indicator */
  function isConnected() {
    return (
      <div
        className="status-indicator"
        style={{ color: serverConnected ? 'lightgreen' : 'red' }}
      >
        {serverConnected ? 'CONNECTED' : 'DISCONNECTED'}
      </div>
    );
  }

  /* Check user authentication */
  useEffect(() => {
    checkAuth(serverIp, setLoggedIn, navigate);
  }, [serverIp, setLoggedIn, navigate]);

  /* Handle loggin in */
  const logIn = useCallback(() => {
    const fullPin = pin.join('');
    return axios
      .post(`http://${serverIp}:3080/auth`, { fullPin })
      .then((response) => {
        if (response.data.message === 'success') {
          const userData = {
            name: response.data.name,
            role: response.data.role,
            token: response.data.token,
          };
          localStorage.setItem('user', JSON.stringify(userData));
          setLoggedIn(true);
          navigate('/home');

          /* Logout after token expires (one hour) */
          setTimeout(
            () => {
              if (loggedIn) {
                setLoggedIn(false);
                navigate('/');
              }
            },
            1 * 1000 * 60 * 60,
          ); // 1 hour
        }
        setPinError('Invalid pin');
      })
      .catch((error) => {
        setPinError(error.message || 'Error during authentication');
      });
  }, [serverIp, loggedIn, pin, setLoggedIn, navigate]);

  /* Handle pin inputs from the buttons */
  const handleKeyClick = (key) => {
    setActiveKey(key);
    if (key === '🡄') {
      setPin((prevPin) => prevPin.slice(0, -1));
    } else if (key === '✔') {
      setPin([]);
      logIn();
    } else if (pin.length < MAX_INPUT_LENGTH) {
      setPin((prevPin) => [...prevPin, key]);
    }
    setTimeout(() => setActiveKey(null), 200);
  };

  /* Handle pin inputs from the keyboard */
  useEffect(() => {
    const handleKeyPress = (event) => {
      if (document.activeElement.id === 'ip-input') return;

      const { key } = event;
      if (/^[0-9]$/.test(key)) {
        if (pin.length < MAX_INPUT_LENGTH) {
          setPin((prevPin) => [...prevPin, key]);
          setActiveKey(key);
        }
      } else if (key === 'Backspace') {
        setPin((prevPin) => prevPin.slice(0, -1));
        setActiveKey('🡄');
      } else if (key === 'Enter') {
        logIn();
        setActiveKey('✔');
      }
      setTimeout(() => setActiveKey(null), 100);
    };

    document.addEventListener('keydown', handleKeyPress);
    return () => {
      document.removeEventListener('keydown', handleKeyPress);
    };
  }, [logIn, pin]); // Only include 'pin' as a dependency if needed

  /* Render circles to represent the pin entered */
  const pinDisplay = pin.map((circle) => (
    <div key={circle.id} className="circle-container">
      <div className="circle" />
    </div>
  ));

  return (
    <div className="Login">
      <div className="server-status">
        <div className="server-connection">
          Server Connection: {isConnected()}
        </div>
        <div className="server-ip">
          Server IP:
          <input
            id="ip-input"
            type="text"
            value={ipAddress}
            onChange={(event) => setIpAddress(event.target.value)}
            onBlur={() => setServerIp(ipAddress)}
            className="ip-input"
          />
        </div>
      </div>
      <div className="login-container">
        <div className="login-header">
          <img className="login-logo" src={logo} alt="Logo" />
        </div>
        <div className="login-box-container">
          <div className="login-box">{pinDisplay}</div>
        </div>
        <div className="keypad-container">
          <KeyPad handleKeyClick={handleKeyClick} activeKey={activeKey} />
        </div>
        {pinError && <div className="pin-error">{pinError}</div>}
      </div>
    </div>
  );
}
Login.propTypes = {
  serverConnected: PropTypes.bool.isRequired,
  serverIp: PropTypes.string.isRequired,
  setServerIp: PropTypes.func.isRequired,
  loggedIn: PropTypes.bool.isRequired,
  setLoggedIn: PropTypes.func.isRequired,
};

export default Login;
