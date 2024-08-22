/* Styling */
import './DroneModule.css';

/* Default Import */
import PropTypes from 'prop-types';

/* Components */
import FormatTF from '../../FormatTF';

function DroneModule({
  droneConnected,
  currentGAltitude,
  currentAltitude,
  targetAltitude,
  currentVelocity,
  targetVelocity,
  armed,
  mode,
  battery,
}) {
  let statusColor;
  if (!droneConnected && !armed) {
    statusColor = '#F00';
  } else if (droneConnected && !armed) {
    statusColor = '#0037ff';
  } else if (armed) {
    statusColor = '#44FF34';
  }
  return (
    <div className="DroneModule">
      <div className="header">
        <div>Drone Status</div>
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="16"
          height="15"
          viewBox="0 0 16 15"
          fill="none"
        >
          <circle cx="8.17273" cy="7.5" r="7.5" fill={statusColor} />
        </svg>
      </div>
      <div className="line" />
      <div className="mod-container">
        {/* Module Section 1 */}
        <div className="mod-section">
          <div className="position-tele">
            <div className="header">Position</div>
            <div className="mod-body">
              <div>
                Current Altitude: <strong>{currentAltitude} </strong>(m)
              </div>
              <div>
                Target Altitude: <strong>{targetAltitude} </strong>(m)
              </div>
              <div>
                Current Velocity: <strong>{currentVelocity} </strong>(m/s)
              </div>
              <div>
                Target Velocity:{' '}
                <strong>
                  {targetVelocity === 0 && mode === 'OFFBOARD'
                    ? 3
                    : targetVelocity}{' '}
                </strong>
                (m/s)
              </div>
            </div>
          </div>
        </div>
        <div className="v-line" />
        {/* Module Section 2 */}
        <div className="mod-section">
          <div className="status-tele">
            <div className="header">Status</div>
            <div className="mod-body">
              <div>
                Armed: <FormatTF value={armed} hasText />
              </div>
              <div>
                Mode: <strong>{mode}</strong>
              </div>
              <div>
                Battery: <strong>{battery}</strong>%
              </div>
              <div>
                Global Altitude: <strong>{currentGAltitude} </strong>(m)
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
DroneModule.propTypes = {
  droneConnected: PropTypes.bool.isRequired,
  currentGAltitude: PropTypes.string.isRequired,
  currentAltitude: PropTypes.string.isRequired,
  targetAltitude: PropTypes.string.isRequired,
  currentVelocity: PropTypes.string.isRequired,
  targetVelocity: PropTypes.string.isRequired,
  armed: PropTypes.bool.isRequired,
  mode: PropTypes.string.isRequired,
  battery: PropTypes.string.isRequired,
};
export default DroneModule;
