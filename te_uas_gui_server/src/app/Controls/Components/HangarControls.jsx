/* Styling */
import './HangarControls.css';

/* Default Imports */
import PropTypes from 'prop-types';
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

function HangarControls({
  openHangar,
  hangarState,
  closeHangar,
  rosConnected,
}) {
  return (
    <div className="HangarControls">
      <div className="controls-header">Hangar Controls</div>
      <div className="line" />
      <div className="controls-buttons">
        <button
          type="button"
          className="controls-button open"
          onClick={openHangar}
          disabled={hangarState}
        >
          Open Hangar
        </button>
        <button
          type="button"
          className="controls-button close"
          onClick={closeHangar}
          disabled={!hangarState}
        >
          Close Hangar
        </button>
      </div>
    </div>
  );
}
HangarControls.propTypes = {
  openHangar: PropTypes.func.isRequired,
  hangarState: PropTypes.bool.isRequired,
  closeHangar: PropTypes.func.isRequired,
  rosConnected: PropTypes.bool.isRequired,
};
export default HangarControls;
