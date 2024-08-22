import './Navbar.css';

import PropTypes from 'prop-types';
import React, { useState, useEffect, useCallback } from 'react';
import { Link, useNavigate } from 'react-router-dom';

/* Components */
import Modal from './Components/Modal';

function Navbar({ ip, armed, setLoggedIn }) {
  const navigate = useNavigate();
  // Navigation Handling
  const [activeItem, setActiveItem] = useState('Main');
  const handleItemClick = useCallback(
    (itemName) => {
      switch (itemName) {
        case 'Main':
          navigate('/home');
          break;
        case 'Controls':
          navigate('/controls');
          break;
        case 'Logs':
          navigate('/logs');
          break;
        default:
          navigate('/home');
          break;
      }
      setActiveItem(itemName);
    },
    [navigate],
  );

  // Event listener for Control + Number navigation
  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.ctrlKey) {
        switch (event.key) {
          case '1':
            handleItemClick('Main');
            break;
          case '2':
            handleItemClick('Controls');
            break;
          case '3':
            handleItemClick('Logs');
            break;
          default:
            break;
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleItemClick]);

  // Logout Handling
  function handleLogout() {
    localStorage.clear();
    setLoggedIn(false);
    navigate('/');
  }
  // Power Button/Modal Handling
  const [modalVisibility, setModalVisibility] = useState(false);
  function handlePowerButton() {
    if (armed) {
      setModalVisibility(true);
    } else {
      localStorage.clear();
      setLoggedIn(false);
      navigate('/');
    }
  }
  return (
    <div className="Navbar">
      <div className="logo">
        <img src={require('../../../assets/MDR-logo.png')} alt="MDR logo" />
      </div>
      <div className="nav">
        <div
          className={activeItem === 'Main' ? 'active' : ''}
          onClick={() => handleItemClick('Main')}
        >
          <Link className="nav-item">Main</Link>
        </div>
        <div
          className={activeItem === 'Logs' ? 'active' : ''}
          onClick={() => handleItemClick('Logs')}
        >
          <Link className="nav-item">Logs</Link>
        </div>
        <div
          className={activeItem === 'Controls' ? 'active' : ''}
          onClick={() => handleItemClick('Controls')}
        >
          <Link className="nav-item">Controls</Link>
        </div>
        <div className="nav-item">Payloads</div>
      </div>
      <div className="navbar-info">
        <StopWatch armed={armed} />
        <Time />
      </div>
      <div className="settings-dropdown">
        <div className="settings-button">
          <button type="button">⚙️</button>
        </div>
        <div className="dropdown-content">
          <button
            type="button"
            className="dropdown-item"
            onClick={() => navigate('/settings')}
          >
            Settings
          </button>
          <button type="button" className="dropdown-item">
            About
          </button>
          <button
            type="button"
            className="dropdown-logout"
            onClick={handleLogout}
          >
            LOGOUT
          </button>
          <button
            type="button"
            className="dropdown-power"
            onClick={handlePowerButton}
          >
            POWER OFF
          </button>
        </div>
      </div>
      <Modal
        ip={ip}
        visible={modalVisibility}
        setVisible={setModalVisibility}
        setLoggedIn={setLoggedIn}
        onClose={() => setModalVisibility(false)}
      />
    </div>
  );
}
Navbar.propTypes = {
  ip: PropTypes.string.isRequired,
  armed: PropTypes.bool.isRequired,
  setLoggedIn: PropTypes.func.isRequired,
};
export default Navbar;

// Functional Component for a stopwatch, which is used to track the flight time.
function StopWatch({ armed }) {
  const [time, setTime] = useState(0);
  // Use state for pausing the timer.
  const [isActive, setIsActive] = useState(false);

  useEffect(() => {
    let interval = null;
    if (isActive) {
      interval = setInterval(() => {
        setTime((prevTime) => prevTime + 1);
      }, 1000);
    } else if (!isActive && time !== 0) {
      clearInterval(interval);
    }
    return () => clearInterval(interval);
  }, [isActive, time]);

  // Automatically activates the stopwatch timer when the drone is armed
  useEffect(() => {
    setIsActive(armed);
    // Reset the time when the drone is unarmed then rearmed
    if (!armed) {
      resetTimer();
    }
  }, [armed]);

  const toggleTimer = () => {
    setIsActive(!isActive);
  };

  const resetTimer = () => {
    setTime(0);
  };

  const formatTime = (timeInSeconds) => {
    const hours = Math.floor(timeInSeconds / 3600);
    const minutes = Math.floor((timeInSeconds % 3600) / 60);
    const seconds = timeInSeconds % 60;

    return `${hours.toString().padStart(2, '0')}h ${minutes.toString().padStart(2, '0')}m ${seconds.toString().padStart(2, '0')}s`;
  };

  return (
    <button type="button" className="flight-timer" onClick={toggleTimer}>
      Flight Time: {formatTime(time)}
    </button>
  );
}
StopWatch.propTypes = {
  armed: PropTypes.bool.isRequired,
};

// Functional Component for displaying the current time
function Time() {
  const [currentTime, setCurrentTime] = useState(new Date());

  useEffect(() => {
    const intervalId = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);

    return () => {
      clearInterval(intervalId);
    };
  }, []);

  // Time needs to be converted from current time the correct 12 hour format
  const formatTime = (time) => {
    let hours = time.getHours();
    const ampm = hours >= 12 ? 'PM' : 'AM';
    hours = hours % 12 || 12; // Convert 0 to 12 for 12-hour format
    const minutes = time.getMinutes().toString().padStart(2, '0');

    return `${hours}:${minutes} ${ampm}`;
  };

  return <p>{formatTime(currentTime)}</p>;
}
