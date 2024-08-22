/* Styling */
import './Home.css';
/* Packages */
import PropTypes from 'prop-types';
import React, { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import ROSLIB from 'roslib';
import axios from 'axios';
/* Components */
import Main from './Main/Main';

export default function Home({
  ip,
  serverIp,
  rosConnected,
  setRosConnected,
  loggedIn,
  armed,
  setArmed,
  setLogs,
}) {
  /* Drone telemetry states */
  const [droneConnected, setDroneConnected] = useState(false); // Drone connection
  const [gAltitude, setGAltitude] = useState('0');
  const [altitude, targetAltitude] = useState('0'); // Altitude of the drone
  const [desAltitude, setdesAltitude] = useState('0'); // Currently Target Altitude
  const [velocity, targetVelocity] = useState('0'); // Velocity of the drone
  const [desVelocity, setdesVelocity] = useState('0'); // Currently Target Velocity
  const [mode, setMode] = useState('OFFBOARD'); // Drone Flight Mode
  const [battery, setBattery] = useState('100'); // Battery Percentage

  /* Hangar telemetry states */
  const [tetherUnspooled, setTetherUnspooled] = useState('0'); // Amount of tether released
  const [unspoolVelocity, setunspoolVelocity] = useState('0'); // Speed of the tether released
  const [hangarState, setHangarState] = useState(false); // Hangar state (opened/closed)

  /* Home Settings */
  const throttlerate = 1000; // Base rate (ms) in which messages are received
  const sigfig = 2; // Number of significant figures for rounding telemetry data

  // const navigate = useNavigate();
  // useEffect(() => {
  //   if (!loggedIn) {
  //     navigate('/');
  //   }
  // }, [loggedIn, navigate]);

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

    /* Subscribe to ROS topics for telemetry data */

    // Position Topic (to get altitude)
    const localPositionSub = new ROSLIB.Topic({
      ros,
      name: '/mavros/local_position/pose',
      messageType: 'geometry_msgs/PoseStamped',
      throttle_rate: throttlerate - 800,
    });
    // Callback function for position
    localPositionSub.subscribe(function (message) {
      // console.log(message);
      targetAltitude(message.pose.position.z.toFixed(sigfig));
    });

    // Position Topic (to get altitude)
    // const localPositionSub = new ROSLIB.Topic({
    //     ros,
    //     name: '/mavros/global_position/global',
    //     messageType: 'sensor_msgs/NavSatFix',
    //     throttle_rate: throttlerate - 800,
    // });
    // // Callback function for position
    // localPositionSub.subscribe(function(message) {
    //     // console.log(message);
    //     targetAltitude(message.altitude.toFixed(sigfig));
    // })

    const globaltolocalPositionSub = new ROSLIB.Topic({
      ros,
      name: '/mavros/global_position/local',
      messageType: 'nav_msgs/msg/Odometry',
      throttle_rate: throttlerate - 800,
    });
    // Callback function for position
    globaltolocalPositionSub.subscribe((message) => {
      setGAltitude(message.pose.pose.position.z.toFixed(sigfig));
    });

    // Set Position Topic
    const deslocalPositionSub = new ROSLIB.Topic({
      ros,
      name: '/mavros/setpoint_position/local',
      messageType: 'geometry_msgs/PoseStamped',
      throttle_rate: throttlerate,
    });
    // Callback function for setdesAltitude
    deslocalPositionSub.subscribe((message) => {
      setdesAltitude(message.pose.position.z.toFixed(0));
    });

    // Velocity Topic
    const localVelocitySub = new ROSLIB.Topic({
      ros,
      name: '/mavros/local_position/velocity_local',
      messageType: 'geometry_msgs/TwistStamped',
      throttle_rate: throttlerate - 800,
    });
    // Callback for Velocity
    localVelocitySub.subscribe(function (message) {
      targetVelocity(message.twist.linear.z.toFixed(sigfig));
    });

    // Target Velocity Topic
    const deslocalVelocitySub = new ROSLIB.Topic({
      ros,
      name: '/mavros/setpoint_velocity/cmd_vel',
      messageType: 'geometry_msgs/TwistStamped',
      throttle_rate: throttlerate,
    });
    // Callback for setdesVelocity
    deslocalVelocitySub.subscribe(function (message) {
      setdesVelocity(message.twist.linear.z.toFixed(0));
    });

    // State (Armed & Mode) Topic
    const stateSub = new ROSLIB.Topic({
      ros,
      name: '/mavros/state',
      messageType: '/mavros_msgs/State',
      throttle_rate: throttlerate,
    });
    // Callback for state
    stateSub.subscribe(function (message) {
      setDroneConnected(message.connected);
      setArmed(message.armed);
      setMode(message.mode);
    });

    // Battery Information Topic
    const batterySub = new ROSLIB.Topic({
      ros,
      name: '/mavros/battery',
      messageType: 'sensor_msgs/BatteryState',
      throttle_rate: throttlerate,
    });
    batterySub.subscribe(function (message) {
      const batteryPercentage = message.percentage * 100;
      setBattery(batteryPercentage.toFixed(0));
    });

    /* Hangar Subsriptions */
    const tetherUnspooledSub = new ROSLIB.Topic({
      ros,
      name: 'hangar/tether_unspooled',
      messageType: 'std_msgs/Float64',
      throttle_rate: throttlerate,
    });
    tetherUnspooledSub.subscribe(function (message) {
      setTetherUnspooled(message.data.toFixed(sigfig));
    });

    const unspoolVelocitySub = new ROSLIB.Topic({
      ros,
      name: 'hangar/tether_unspool_speed',
      messageType: 'std_msgs/Float64',
      throttle_rate: throttlerate,
    });
    unspoolVelocitySub.subscribe(function (message) {
      setunspoolVelocity(message.data.toFixed(sigfig));
    });

    /* Notification Subscription */

    const notificationSub = new ROSLIB.Topic({
      ros,
      name: 'te_uas/logger',
      messageType: 'te_uas_msgs/Log',
      throttle_rate: 0, // No throttle becuase notifications are burst but infrequent
    });
    notificationSub.subscribe(() => {
      const getActiveLogs = async () => {
        const user = JSON.parse(localStorage.getItem('user'));
        if (user) {
          try {
            const response = await axios.get(
              `http://${serverIp}:3081/flights?active=true`,
              {
                headers: { 'jwt-token': user.token },
              },
            );

            if (response.data && response.data.logs) {
              setLogs(response.data.logs);
            }
          } catch (error) {
            // console.error('Error fetching active logs:', error);
          }
        }
      };

      setTimeout(() => {
        getActiveLogs();
      }, 500);
    });

    // Clean up connection on unmount
    return () => {
      ros.close();
    };
  }, [ip, serverIp, setArmed, setRosConnected, setLogs]);

  return (
    <div className="Home">
      {loggedIn && (
        <Main
          ip={ip}
          serverIp={serverIp}
          rosConnected={rosConnected}
          droneConnected={droneConnected}
          gAltitude={gAltitude}
          altitude={altitude}
          desAltitude={desAltitude}
          velocity={velocity}
          desVelocity={desVelocity}
          armed={armed}
          mode={mode}
          battery={battery}
          tetherUnspooled={tetherUnspooled}
          unspoolVelocity={unspoolVelocity}
          hangarState={hangarState}
          // setMode={setMode()}
        />
      )}
    </div>
  );
}
Home.propTypes = {
  ip: PropTypes.string.isRequired,
  serverIp: PropTypes.string.isRequired,
  rosConnected: PropTypes.bool.isRequired,
  setRosConnected: PropTypes.func.isRequired,
  loggedIn: PropTypes.bool.isRequired,
  armed: PropTypes.bool.isRequired,
  setArmed: PropTypes.func.isRequired,
  setLogs: PropTypes.func.isRequired,
};
