/* Styling */
import './Controls.css';

/* Default Imports */
import PropTypes from 'prop-types';
import React, { useEffect, useState, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import ROSLIB from 'roslib';

/* Components */
import DroneControls from './Components/DroneControls';
import HangarControls from './Components/HangarControls';

/* Component for the Controls Tab */
function Controls({ ip, loggedIn }) {
  const [rosConnected, setRosConnected] = useState(false);
  const [hangarState, setHangarState] = useState(false); // Hangar state (opened/closed)

  // const navigate = useNavigate();
  // useEffect(() => {
  //   if (!loggedIn) {
  //     navigate('/');
  //   }
  // }, [loggedIn, navigate]);

  /* Connect to ROS when page is clicked */
  useEffect(() => {
    /* Connect to ROS */
    const ros = new ROSLIB.Ros({
      url: `ws://${ip}:9090`,
    });
    ros.on('connection', () => {
      console.log('Connected to ROS');
      setRosConnected(true);
    });

    ros.on('error', (error) => {
      console.log('Error connecting to ROS: ', error);
      setRosConnected(false);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed');
      setRosConnected(false);
    });

    // Clean up connection on unmount
    return () => {
      ros.close();
    };
  }, []);

  const openHangar = () => {
    const ros = new ROSLIB.Ros({
      url: `ws://${ip}:9090`,
    });
    ros.on('connection', () => {
      console.log('Connected to ROS');
      setRosConnected(true);
    });

    ros.on('error', (error) => {
      console.log('Error connecting to ROS: ', error);
      setRosConnected(false);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed');
      setRosConnected(false);
    });

    const doorsService = new ROSLIB.Service({
      ros,
      name: 'te_uas/doors_srvc',
      serviceType: 'te_uas_msgs/ControlDoor',
    });
    const request = new ROSLIB.ServiceRequest({
      open: true,
    });
    doorsService.callService(request, (response) => {
      console.log(response);
      ros.close();
    });
  };

  const closeHangar = () => {
    const ros = new ROSLIB.Ros({
      url: `ws://${ip}:9090`,
    });
    ros.on('connection', () => {
      console.log('Connected to ROS');
      setRosConnected(true);
    });

    ros.on('error', (error) => {
      console.log('Error connecting to ROS: ', error);
      setRosConnected(false);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed');
      setRosConnected(false);
    });

    const doorsService = new ROSLIB.Service({
      ros,
      name: 'te_uas/doors_srvc',
      serviceType: 'te_uas_msgs/ControlDoor',
    });
    const request = new ROSLIB.ServiceRequest({
      open: false,
    });
    doorsService.callService(request, (response) => {
      console.log(response);
      ros.close();
    });
  };

  return (
    <div className="Controls">
      <div className="controls-container">
        <HangarControls />
        <div className="v-line" />
        <DroneControls />
      </div>
    </div>
  );
}
Controls.propTypes = {
  ip: PropTypes.string.isRequired,
  loggedIn: PropTypes.bool.isRequired,
};
export default Controls;
