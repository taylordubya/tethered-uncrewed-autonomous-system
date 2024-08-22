/* Styling */
import './Settings.css';

/* Default Imports */
import PropTypes from 'prop-types';
import React, { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';

/* Imported Packages */
import axios from 'axios';

/* Components */
import AdminSection from './Components/AdminSection';
import SettingsSection from './Components/SettingsSection';

function Settings({ serverIp, ip, setIp, loggedIn, setLoggedIn }) {
  const navigate = useNavigate();
  const [activeItem, setActiveItem] = useState('Settings'); // State variable to track active item
  const user = JSON.parse(localStorage.getItem('user'));

  useEffect(() => {
    if (!loggedIn) {
      navigate('/');
    }
  }, [loggedIn]);

  // Logout Handling
  const handleLogout = () => {
    localStorage.clear();
    setLoggedIn(false);
    navigate('/');
  };

  return (
    <div className="Settings">
      <div className="settings-container">
        <div className="settings-menu-container">
          <div className="settings-menu">
            <div className="settings-profile">
              <div className="user-pic" />
              <div className="user-info">
                <div className="header">{user.name}</div>
                <div className="sub-header">
                  <strong>Role:</strong> {user.role}
                </div>
              </div>
            </div>
            <div className="settings-nav">
              <div
                className={`nav-item ${activeItem === 'Settings' ? 'active' : ''}`}
                onClick={() => setActiveItem('Settings')}
              >
                Settings
              </div>
              {user.role === 'Administrator' && (
                <div
                  className={`nav-item ${activeItem === 'Admin' ? 'active' : ''}`}
                  onClick={() => setActiveItem('Admin')}
                >
                  Admin
                </div>
              )}
              <div className="power-settings">
                <button
                  type="button"
                  className="nav-item log-out"
                  onClick={handleLogout}
                >
                  Logout
                </button>
                <button type="button" className="nav-item power-off">
                  Power Off
                </button>
              </div>
            </div>
          </div>
        </div>
        <div className="v-line" />
        {activeItem === 'Settings' && <SettingsSection serverIp={serverIp} />}
        {activeItem === 'Admin' && user.role === 'Administrator' && (
          <AdminSection serverIp={serverIp} user={user} />
        )}
      </div>
    </div>
  );
}
Settings.propTypes = {
  serverIp: PropTypes.string.isRequired,
  ip: PropTypes.string.isRequired,
  setIp: PropTypes.string.isRequired,
  setLoggedIn: PropTypes.func.isRequired,
};
export default Settings;
