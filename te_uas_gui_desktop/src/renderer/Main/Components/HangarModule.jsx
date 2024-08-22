// Hangar Module
import './HangarModule.css';

/* Default */
import PropTypes from 'prop-types';

/*  */
import FormatTF from '../../FormatTF';

function HangarModule({
  rosConnected,
  tetherUnspooled,
  unspoolVelocity,
  hangarState,
}) {
  return (
    <div className="HangarModule">
      <div className="header">
        <div>Hangar Status</div>
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
      <div className="m3-container">
        <div className="m3-section">
          <div className="hangar-tele">
            <div className="header">Info</div>
            <div className="mod-body">
              <div>
                Hangar State:{' '}
                <FormatTF
                  trueText="OPENED"
                  falseText="CLOSED"
                  value={hangarState}
                  hasText
                />
              </div>
              <div>
                Tether Released: <strong>{tetherUnspooled} </strong>(m)
              </div>
              <div>
                Unspool Speed: <strong>{unspoolVelocity} </strong>(m/s)
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

HangarModule.propTypes = {
  rosConnected: PropTypes.bool.isRequired,
  tetherUnspooled: PropTypes.string.isRequired,
  unspoolVelocity: PropTypes.string.isRequired,
  hangarState: PropTypes.bool.isRequired,
};
export default HangarModule;
