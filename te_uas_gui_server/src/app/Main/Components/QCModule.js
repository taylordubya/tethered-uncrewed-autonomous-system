/* Styling */
import './Modules.css';
import './QCModule.css';

/* Default Modules */
import PropTypes from 'prop-types';
import { useState, useEffect, useCallback } from 'react';
import ROSLIB from 'roslib';

/* Imported Packages */
import axios from 'axios';

/* Components */
import AltitudeModal from './AltitudeModal';

// Create list of drone and camera modes to be dynamically generated for the dropdown
// const droneModes = ['OFFBOARD', 'AUTO.LOITER', 'AUTO.LAND'];
const cameraModes = ['FIXED', 'PAN RIGHT', 'PAN LEFT', 'SURVEY'];
// Functional Component to dynamically create dropdowns when given a list
function DropDown({ ip, options }) {
  // Handle drop down
  const handleDropDown = (event) => {
    // Value selected from dropdown
    const selectedOption = event.target.value;
    const ros = new ROSLIB.Ros({
      url: `ws://${ip}:9090`, // Replace with your ROS WebSocket server URL
      // I had to manually set the IPs on both devices to get this to work...
    });
    ros.on('connection', () => {
      // console.log('Connected to ROS');
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    ros.on('close', () => {
      console.log('Disconnected from ROS');
    });

    const setModeService = new ROSLIB.Service({
      ros,
      name: 'mavros/set_mode',
      serviceType: 'mavros_msgs/SetMode',
    });

    const request = new ROSLIB.ServiceRequest({
      base_mode: 0,
      custom_mode: selectedOption,
    });

    setModeService.callService(request, (response) => {
      console.log('Set mode service response: ', response);
      ros.close();
    });
  };

  // Create dropdown for every option listed
  const dropdownOptions = options.map((option) => (
    <option key={option.id} value={option}>
      {option}
    </option>
  ));

  return (
    <select className="dropdown" onChange={handleDropDown}>
      {dropdownOptions}
    </select>
  );
}
DropDown.propTypes = {
  ip: PropTypes.string.isRequired,
  options: PropTypes.arrayOf(PropTypes.string).isRequired,
};

// Functional Component that creates a controller to control the camera
function DPad() {
  const handleUp = () => {
    console.log('Up button clicked');
  };

  const handleDown = () => {
    console.log('Down button clicked');
  };

  const handleLeft = () => {
    console.log('Left button clicked');
  };

  const handleRight = () => {
    console.log('Right button clicked');
  };

  const handleMiddle = () => {
    console.log('Middle button clicked');
  };

  return (
    <div className="dpad-container">
      <div className="dpad-row">
        <button
          className="dbutton up"
          type="button"
          aria-label="button up"
          onClick={handleUp}
        />
      </div>
      <div className="dpad-row">
        <button
          className="dbutton left"
          type="button"
          aria-label="button left"
          onClick={handleLeft}
        />
        <button
          className="dbutton middle"
          type="button"
          aria-label="button middle"
          onClick={handleMiddle}
        />
        <button
          className="dbutton right"
          type="button"
          aria-label="button right"
          onClick={handleRight}
        />
      </div>
      <div className="dpad-row">
        <button
          className="dbutton down"
          type="button"
          aria-label="button down"
          onClick={handleDown}
        />
      </div>
    </div>
  );
}

// Functional Component that creates a slider for the camera zoom
function Slider() {
  const [sliderValue, setSliderValue] = useState(50);

  const handleSliderChange = (event) => {
    setSliderValue(event.target.value);
  };

  return (
    <div className="zoom-container">
      <div className="camera-header">Zoom</div>
      <input
        type="range"
        min="0"
        max="100"
        value={sliderValue}
        onChange={handleSliderChange}
      />
      <div>{sliderValue}%</div>
    </div>
  );
}

function QuickControlModule({ ip, serverIp, rosConnected, armed }) {
  const [customAltitude, setCustomAltitude] = useState(null);
  const [isModalVisible, setIsModalVisible] = useState(false);

  /* Button Handling, calls ROS services to perform predefined flight actions */
  const handleLand = () => {
    const ros = new ROSLIB.Ros({
      url: `ws://${ip}:9090`, // Replace with your ROS WebSocket server URL
      // I had to manually set the IPs on both devices to get this to work...
    });
    ros.on('connection', () => {
      console.log('Connected to ROS');
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    ros.on('close', () => {
      console.log('Disconnected from ROS');
    });

    const setModeService = new ROSLIB.Service({
      ros,
      name: 'mavros/set_mode',
      serviceType: 'mavros_msgs/SetMode',
    });
    const request = new ROSLIB.ServiceRequest({
      base_mode: 0,
      custom_mode: 'AUTO.LAND',
    });
    setModeService.callService(request, (response) => {
      console.log('Set mode service response: ', response);

      const user = JSON.parse(localStorage.getItem('user'));

      const endFlight = async () => {
        if (user) {
          try {
            const serverResponse = await axios.post(
              `http://${serverIp}:3081/flight`,
              { action: 'end', user },
              {
                headers: { 'jwt-token': user.token },
              },
            );

            if (serverResponse.status === 200) {
              console.log('Flight ended and logged successfully');
            } else {
              console.log('Failed to end flight');
            }
          } catch (error) {
            // console.log(
            //   `Error: ${error.response ? error.response.data.message : error.message}`,
            // );
          }
        } else {
          console.log('No user found in local storage.');
        }
      };

      // Call the endFlight function with the server IP
      endFlight(serverIp);
    });
  };

  const handleLaunch = useCallback(() => {
    if (rosConnected && !armed) {
      const user = JSON.parse(localStorage.getItem('user'));

      const createFlight = async () => {
        if (user) {
          const flightInfo = {
            action: 'start',
            info: { tookoffBy: user.name },
            user,
          };

          try {
            await axios.post(`http://${serverIp}:3081/flight`, flightInfo, {
              headers: { 'jwt-token': user.token },
            });
            // console.log('Flight created successfully');
          } catch (error) {
            /* Empty */
          }
        } else {
          // console.log('No user found in local storage.');
        }
      };

      // Call the createFlight function with the server IP
      createFlight(serverIp);
    }

    const ros = new ROSLIB.Ros({
      url: `ws://${ip}:9090`, // Replace with your ROS WebSocket server URL
    });

    ros.on('connection', () => {
      console.log('Connected to ROS');
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    ros.on('close', () => {
      console.log('Disconnected from ROS');
    });

    const remoteFlyService = new ROSLIB.Service({
      ros,
      name: 'te_uas/adjust_alt_srvc',
    });

    const customAltitudeRequest = new ROSLIB.ServiceRequest({
      min_pitch: 0.0,
      yaw: 0.0,
      latitude: 0.0,
      longitude: 0.0,
      altitude: parseFloat(customAltitude),
    });

    remoteFlyService.callService(customAltitudeRequest, (response) => {
      console.log('adjust_alt_srvc response: ', response);
      ros.close();
    });
  }, [rosConnected, armed, serverIp, ip, customAltitude]);

  const handleLaunchClick = () => {
    if (!isModalVisible) {
      setIsModalVisible(true);
    } else {
      handleLaunch();
    }
  };

  /* Open Modal when enter is pressed */
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.key === 'Enter') {
        if (isModalVisible) {
          setIsModalVisible(false);
          handleLaunch();
        } else {
          setIsModalVisible(true);
        }
      }
      if (e.key === 'Escape') {
        setIsModalVisible(false);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [isModalVisible, handleLaunch]);

  /* Handle the Quick Altitude Adjustment box functionality */
  function handleAltitudeChange(e) {
    if (e.target.value > 100) e.target.value = 100; // Keep set altitude under 100(m)
    if (e.target.value < 0) e.target.value = 0;
    setCustomAltitude(e.target.value);
  }
  /* Handle the Quick Velocity Adjustment box functionality */
  const handleVelocityChange = (e) => {
    let { value } = e.target;
    if (value > 10) value = 10;
    setCustomAltitude(value);
  };
  return (
    <div className="QCModule">
      <div className="header">
        <div>Quick Controls</div>
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="16"
          height="15"
          viewBox="0 0 16 15"
          fill="none"
        >
          <circle
            cx="8.17273"
            cy="7.5"
            r="7.5"
            fill={rosConnected ? '#44FF34' : '#F00'}
          />
        </svg>
      </div>
      <div className="line" />
      {/* Module Section */}
      <div className="m1-container">
        <div className="m1-section">
          <div className="m1-header">Change Modes</div>
          <div className="mode-body">
            {/* <div>Set Drone Mode: </div>
                        <DropDown ip={ip} options={droneModes}/> */}
            <div>Set Camera Mode: </div>
            <DropDown ip={ip} options={cameraModes} />
          </div>
          <div className="line" />
        </div>

        {/* Module Section 2 */}
        <div className="m1-section">
          <div className="cc-container">
            <div>
              <div className="m1-header">Camera Controls</div>
            </div>
            <DPad />
            <div />
          </div>
          <Slider />
          <div className="cc-description">
            Control the camera with: WASD/🡅🡄🡇🡆
          </div>
          <div className="line" />
        </div>
        {/* Module Section 3 */}
        <div className="m1-section">
          <div className="m1-header">Quick Commands</div>
          <div className="buttons">
            <button type="button" onClick={handleLaunchClick}>
              LAUNCH
            </button>
            <button type="button" onClick={handleLand}>
              LAND
            </button>
          </div>
          <div className="line" />
        </div>
      </div>
      <AltitudeModal
        isVisible={isModalVisible}
        setIsVisible={setIsModalVisible}
        customAltitude={customAltitude}
        handleAltitudeChange={handleAltitudeChange}
        handleLaunch={handleLaunch}
      />
    </div>
  );
}
QuickControlModule.propTypes = {
  ip: PropTypes.string.isRequired,
  serverIp: PropTypes.string.isRequired,
  rosConnected: PropTypes.bool.isRequired,
  armed: PropTypes.bool.isRequired,
};
export default QuickControlModule;
