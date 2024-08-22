import './Logs.css';
import PropTypes from 'prop-types';
import { useEffect, useState } from 'react';
import axios from 'axios';

export function styling(log) {
  if (log.level === 'CRITICAL') {
    return '#FF0000';
  }
  if (log.level === 'WARNING') {
    return '#EED202';
  }
  if (log.description.includes('Adjusting altitude')) {
    return '#008000';
  }
  if (log.description.includes('changed mode to AUTO.LAND')) {
    return '#0096FF';
  }
  return '#000';
}

async function getAllFlights(serverIp, setAllFlights) {
  try {
    const response = await axios.get(`http://${serverIp}:3081/flights`);
    setAllFlights(response.data);
  } catch (error) {
    console.error('Error fetching flights:', error);
  }
}

function Logs({ setNotificationVisible, serverIp, logs }) {
  const [displayedFlight, setDisplayedFlight] = useState({ id: 0, logs });
  const [checkedLogs, setCheckedLogs] = useState([]);
  const [allFlights, setAllFlights] = useState([]);
  const [filterPage, setFilterPage] = useState(1);

  useEffect(() => {
    if (allFlights.length > 0) {
      setDisplayedFlight(allFlights[allFlights.length - 1]);
    }
  }, [allFlights]);

  useEffect(() => {
    setNotificationVisible(false);
    getAllFlights(serverIp, setAllFlights);
    return () => {
      setNotificationVisible(true);
    };
  }, [serverIp, setNotificationVisible]);

  const handleCheckboxChange = (level) => {
    if (checkedLogs.includes(level)) {
      setCheckedLogs(checkedLogs.filter((item) => item !== level));
    } else {
      setCheckedLogs([...checkedLogs, level]);
    }
  };

  const filteredItems = displayedFlight.logs.filter(
    (item) => checkedLogs.length === 0 || checkedLogs.includes(item.level),
  );

  const filterOptions = [
    ['CRITICAL', 'WARNING', 'ERROR', 'INFO'],
    ['DRONE', 'HANGAR'],
  ];

  const renderFilterOptions = () => {
    return filterOptions[filterPage - 1].map((option) => (
      <div key={option} className="filter-option">
        <input
          type="checkbox"
          id={`checkbox-${option}`}
          checked={checkedLogs.includes(option)}
          onChange={() => handleCheckboxChange(option)}
        />
        <label htmlFor={`checkbox-${option}`}>{option}</label>
      </div>
    ));
  };

  return (
    <div className="Logs">
      <div className="log-container">
        <div className="log-sidebar">
          <div className="log-filters-container">
            <div className="filters">
              <div className="log-header">Filters</div>
              <div className="filter-line" />
              <div className="filter-options">{renderFilterOptions()}</div>
              <div className="filter-line" />
              <div className="filter-navigation">
                <div className="filter-navigation-buttons">
                  <button
                    className="filter-page-left"
                    onClick={() =>
                      setFilterPage(
                        filterPage === 1
                          ? filterOptions.length
                          : filterPage - 1,
                      )
                    }
                  >
                    &lt;
                  </button>
                  <button
                    className="filter-page-right"
                    onClick={() =>
                      setFilterPage(
                        filterPage === filterOptions.length
                          ? 1
                          : filterPage + 1,
                      )
                    }
                  >
                    &gt;
                  </button>
                </div>
                <div className="filter-page-dots">
                  {filterOptions.map((_, index) => (
                    <span
                      key={index}
                      className={`filter-dot ${filterPage === index + 1 ? 'active' : ''}`}
                    >
                      •
                    </span>
                  ))}
                </div>
              </div>
            </div>
          </div>
          <div className="previous-flights-container">
            <div className="previous-flights">
              <div className="log-header">Previous Flights</div>
              <div className="flights-line" />
              <div className="scrollable">
                <div className="flights">
                  {allFlights &&
                    allFlights.toReversed().map((flight) => (
                      <button
                        type="button"
                        className={`previous-log ${displayedFlight.id === flight.id ? 'active' : ''}`}
                        key={flight.id}
                        onClick={() => setDisplayedFlight(flight)}
                      >
                        <div className="log-title">
                          Flight @ {flight.info.tookoffAt}
                        </div>
                      </button>
                    ))}
                </div>
              </div>
            </div>
          </div>
        </div>
        <div className="log-body-container">
          <div className="log-body">
            <div className="log-body-headers">
              <div className="log-header">Time</div>
              <div className="log-header">Level</div>
              <div className="log-header">Description</div>
            </div>
            <div className="line" />
            <div className="log-list">
              {filteredItems.toReversed().map((item) => (
                <div className="log" key={item.id}>
                  <div className="log-date">{item.timestamp}</div>
                  <div className="v-line" />
                  <div className="log-level" style={{ color: styling(item) }}>
                    {item.level}
                  </div>
                  <div className="v-line" />
                  <div className="log-description">{item.description}</div>
                </div>
              ))}
              <div />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

Logs.propTypes = {
  setNotificationVisible: PropTypes.func.isRequired,
  serverIp: PropTypes.string.isRequired,
  logs: PropTypes.arrayOf(PropTypes.object).isRequired,
};

export default Logs;
