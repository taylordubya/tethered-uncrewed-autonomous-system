/* Styling */
import { useEffect } from 'react';
import './PreviousFlights.css';

/* Default Imports */
import PropTypes from 'prop-types';

function PreviousFlights(
  serverIp,
  allFlights,
  setAllFlights,
  displayedFlightid,
  setDisplayedFlight,
) {
  useEffect(() => {
    console.log(allFlights);
  }, [allFlights, displayedFlightid]);
  return (
    <div className="previous-flights-container">
      <div className="previous-flights">
        <div className="log-header">Previous Flights</div>
        <div className="flights-line" />
        <div className="scrollable">
          <div className="flights">
            {/* {allFlights &&
              allFlights.toReversed().map((flight) => (
                <button
                  type="button"
                  className={`previous-log ${displayedFlightid === flight.id ? 'active' : ''}`}
                  key={flight.id}
                  onClick={() => setDisplayedFlight(flight)}
                >
                  <div className="log-title">
                    Flight @ {flight.info.tookoffAt}
                  </div>
                </button>
              ))} */}
          </div>
        </div>
      </div>
    </div>
  );
}
PreviousFlights.propTypes = {
  serverIp: PropTypes.string.isRequired,
  displayedFlightid: PropTypes.string.isRequired,
  setDisplayedFlight: PropTypes.func.isRequired,
};
export default PreviousFlights;
