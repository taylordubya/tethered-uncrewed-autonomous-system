import PropTypes from 'prop-types';
import axios from 'axios';
import React, { useEffect, useState } from 'react';
import './Notifications.css';

// Function to determine the styling color based on log level or description
export function styling(log) {
  if (log.level === 'CRITICAL') {
    return '#FF0000'; // Red for critical logs
  }
  if (log.level === 'WARNING') {
    return '#EED202'; // Yellow for warning logs
  }
  if (log.description.includes('Adjusting altitude')) {
    return '#008000'; // Green for altitude adjustments
  }
  if (log.description.includes('changed mode to AUTO.LAND')) {
    return '#0096FF'; // Blue for mode change to AUTO.LAND
  }
  return '#000'; // Default color black
}

function Notifications({ notificationVisible, serverIp, logs, setLogs }) {
  const MAX_NOTIFICATION_COUNT = 9;
  const user = JSON.parse(localStorage.getItem('user')); // Get user info from local storage
  const [notifications, setNotifications] = useState(
    logs.filter((log) => !log.dismissed.includes(user.name)),
  );

  // Update notifications when logs or user.name changes
  useEffect(() => {
    setNotifications(logs.filter((log) => !log.dismissed.includes(user.name)));
  }, [logs, user.name]);

  // Function to dismiss a single log
  const dismissLog = async (logId) => {
    if (user) {
      try {
        const response = await axios.post(
          `http://${serverIp}:3081/logs/dismiss`,
          { name: user.name, logId },
          {
            headers: { 'jwt-token': user.token },
          },
        );
        // Update the logs locally after dismissal
        setLogs(response.data.updatedLogs);
      } catch (error) {
        // console.error('Error dismissing log:', error);
      }
    } else {
      // console.log('No user found in local storage.');
    }
  };

  // Function to dismiss all logs for the user
  const dismissAllLogs = async () => {
    if (user) {
      try {
        const response = await axios.post(
          `http://${serverIp}:3081/logs/dismiss`,
          { name: user.name },
          {
            headers: { 'jwt-token': user.token },
          },
        );
        // Update the logs locally after dismissal
        setLogs(response.data.updatedLogs);
      } catch (error) {
        // console.error('Error dismissing all logs:', error);
      }
    } else {
      // console.log('No user found in local storage.');
    }
  };

  // Create a list of notification elements
  const notificationList = notifications
    .slice(-MAX_NOTIFICATION_COUNT)
    .toReversed()
    .map((notification, index) => (
      <button
        id={`notification-${index}`}
        className="notification"
        key={notification.id}
        type="button"
        onClick={() => dismissLog(notification.logId)}
      >
        <svg
          className="notificationIndicator"
          width="30"
          height="28"
          viewBox="0 0 30 28"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M29.6786 24.2721L16.6537 1.71388C16.315 1.12192 15.6794 0.754272 14.9966 0.754272C14.3153 0.754272 13.6803 1.12132 13.3388 1.71175L0.255652 24.3745C-0.0852173 24.9644 -0.0852173 25.6968 0.255652 26.2866C0.596521 26.8781 1.23154 27.2456 1.91237 27.2456H28.0878C29.1422 27.2456 29.9998 26.3867 29.9998 25.3313C30 24.9376 29.8834 24.5585 29.6786 24.2721ZM28.0878 26.1363H1.91298C1.62659 26.1363 1.35861 25.9827 1.21526 25.7338C1.07146 25.4848 1.071 25.176 1.21526 24.9272L14.2984 2.26505C14.4416 2.01731 14.709 1.86255 14.9966 1.86255C15.283 1.86255 15.5526 2.01716 15.6954 2.26505L28.7464 24.8685C28.8417 25.0047 28.8934 25.1642 28.8934 25.332C28.8934 25.7753 28.5312 26.1363 28.0878 26.1363Z"
            fill={styling(notification)}
          />
          <path
            d="M15.5984 19.534L15.9 9.23447H13.6514L13.9514 19.534H15.5984Z"
            fill={styling(notification)}
          />
          <path
            d="M14.7755 21.01C13.9924 21.01 13.4243 21.6042 13.4243 22.4222C13.4243 23.2424 13.9924 23.8365 14.7755 23.8365C15.5712 23.8365 16.1277 23.2559 16.1277 22.4222C16.1277 21.5905 15.5716 21.01 14.7755 21.01Z"
            fill={styling(notification)}
          />
        </svg>
        <div
          className="notificationType"
          style={{ color: styling(notification) }}
        >
          {notification.level}:
        </div>
        <div className="notificationDesc">{notification.description}</div>
        <div className="exit">
          <svg
            width="11"
            height="11"
            viewBox="0 0 11 11"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M6.8061 5.50023L10.8919 1.41468C11.037 1.26959 11.037 1.03481 10.8919 0.889774L10.1111 0.108628C10.0415 0.0391301 9.94713 0 9.84863 0C9.7501 0 9.65572 0.0391301 9.58618 0.108628L5.50036 4.19449L1.41455 0.108628C1.345 0.0391301 1.25063 0 1.1521 0C1.05359 0 0.959195 0.0391301 0.889673 0.108628L0.108813 0.889774C-0.0362711 1.03481 -0.0362711 1.26959 0.108813 1.41468L4.19463 5.50025L0.109219 9.58532C-0.0357937 9.73041 -0.0357937 9.96519 0.109219 10.1102L0.890126 10.8914C0.959625 10.9609 1.05402 11 1.15255 11C1.25108 11 1.34546 10.9609 1.41503 10.8914L5.50034 6.80599L9.58568 10.8914C9.65522 10.9609 9.7496 11 9.84813 11C9.94668 11 10.0411 10.9609 10.1106 10.8914L10.8915 10.1102C11.0365 9.96519 11.0365 9.73041 10.8915 9.58532L6.8061 5.50023Z"
              fill="#BEBEBE"
            />
          </svg>
        </div>
      </button>
    ));

  return (
    <div className="notificationContainer">
      {notificationVisible && notifications.length > 1 && (
        <button
          aria-label="Dismiss All Logs"
          className="exit-all"
          type="button"
          onClick={dismissAllLogs}
        >
          <svg
            width="11"
            height="11"
            viewBox="0 0 11 11"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M6.8061 5.50023L10.8919 1.41468C11.037 1.26959 11.037 1.03481 10.8919 0.889774L10.1111 0.108628C10.0415 0.0391301 9.94713 0 9.84863 0C9.7501 0 9.65572 0.0391301 9.58618 0.108628L5.50036 4.19449L1.41455 0.108628C1.345 0.0391301 1.25063 0 1.1521 0C1.05359 0 0.959195 0.0391301 0.889673 0.108628L0.108813 0.889774C-0.0362711 1.03481 -0.0362711 1.26959 0.108813 1.41468L4.19463 5.50025L0.109219 9.58532C-0.0357937 9.73041 -0.0357937 9.96519 0.109219 10.1102L0.890126 10.8914C0.959625 10.9609 1.05402 11 1.15255 11C1.25108 11 1.34546 10.9609 1.41503 10.8914L5.50034 6.80599L9.58568 10.8914C9.65522 10.9609 9.7496 11 9.84813 11C9.94668 11 10.0411 10.9609 10.1106 10.8914L10.8915 10.1102C11.0365 9.96519 11.0365 9.73041 10.8915 9.58532L6.8061 5.50023Z"
              fill="#BEBEBE"
            />
          </svg>
        </button>
      )}
      {notificationVisible && (
        <div className="notifications">{notificationList}</div>
      )}
    </div>
  );
}

Notifications.propTypes = {
  notificationVisible: PropTypes.bool.isRequired,
  serverIp: PropTypes.string.isRequired,
  logs: PropTypes.arrayOf(PropTypes.objectOf).isRequired,
  setLogs: PropTypes.func.isRequired,
};

export default Notifications;
