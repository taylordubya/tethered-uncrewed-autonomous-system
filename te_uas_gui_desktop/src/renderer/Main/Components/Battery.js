/* Styling */
import './Modules.css';

/* Default Imports */
import PropTypes from 'prop-types';

function Battery({ charge, tethered }) {
  const getBatteryColor = () => {
    if (tethered) {
      return '#303F77'; // Blue when tethered
    }
    if (charge <= 20) {
      return `linear-gradient(90deg,
                hsl(7, 89%, 46%) 15%,
                hsl(11, 93%, 68%) 100%)`; // Red when charge is <= to 20%
    }
    if (charge <= 40) {
      return `linear-gradient(90deg,
                hsl(22, 89%, 46%) 15%,
                hsl(54, 90%, 45%) 100%)`; // Orange when charge is <= to 50%
    }
    if (charge <= 80) {
      return `linear-gradient(90deg,
                hsl(54, 89%, 46%) 15%,
                hsl(92, 90%, 45%) 100%)`; // Yellow when charge is <= to 80%
    }
    return `linear-gradient(90deg,
                hsl(92, 89%, 46%) 15%,
                hsl(92, 90%, 68%) 100%)`; // Green when charge > 50%
  };

  const calculateWidth = () => {
    if (tethered) {
      return '100%';
    }
    return `${charge}%`;
  };

  const getBatteryPercentage = () => {
    if (tethered) {
      return 'TETHERED';
    }
    return `${charge}%`;
  };

  return (
    <div className="battery-container">
      <div className="Battery">
        <div
          className="battery-level"
          style={{ width: calculateWidth(), background: getBatteryColor() }}
        />
        <div className="battery-fill">
          <div className="battery-percentage">{getBatteryPercentage()}</div>
        </div>
      </div>
    </div>
  );
}
Battery.propTypes = {
  charge: PropTypes.string.isRequired,
  tethered: PropTypes.bool.isRequired,
};
export default Battery;
