/* Styling */
import './DroneControls.css';

/* Default Imports */
import PropTypes from 'prop-types';
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

function DroneControls() {
  return (
    <div className="DroneControls">
      <div className="">
        <div className="controls-header">Drone Controls</div>
        <div className="line" />
      </div>
    </div>
  );
}
export default DroneControls;
