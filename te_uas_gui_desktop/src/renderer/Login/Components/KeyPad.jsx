/* Styling */
import './KeyPad.css';

/* Default Imports */
import PropTypes from 'prop-types';

/* Keypad Component */
function KeyPad({ handleKeyClick, activeKey }) {
  const keys = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '🡄', '0', '✔'];
  const keyPadKeys = keys.map((key) => (
    <div key={key.id} className="key-container">
      <button
        type="button"
        className={`key ${activeKey === key ? 'key-active' : ''}`}
        onClick={() => handleKeyClick(key)}
        tabIndex={0}
        onKeyPress={(e) => e.key === 'Enter' && handleKeyClick(key)}
      >
        {key}
      </button>
    </div>
  ));
  return <div className="KeyPad">{keyPadKeys}</div>;
}
KeyPad.defaultProps = {
  activeKey: '', // Default value for activeKey
};
KeyPad.propTypes = {
  handleKeyClick: PropTypes.func.isRequired,
  activeKey: PropTypes.string,
};
export default KeyPad;
