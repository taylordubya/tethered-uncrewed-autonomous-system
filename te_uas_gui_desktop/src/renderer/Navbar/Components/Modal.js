/* Styling */
import './Modal.css';

/* Default Imports */
import PropTypes from 'prop-types';
import { useEffect, useCallback } from 'react';
import ROSLIB from 'roslib';

function Modal({ ip, visible, setVisible, setLoggedIn, onClose }) {
  function handleContinue() {
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

    const stopNodeService = new ROSLIB.Service({
      ros,
      name: 'te_uas/killswitch',
    });
    const stopRequest = new ROSLIB.ServiceRequest({});

    stopNodeService.callService(stopRequest, function (response) {
      console.log('Stop service response: ', response);
      ros.close();
    });

    setVisible(false);
    // setLoggedIn(false);
    // localStorage.clear();
    // navigate('/');
  }
  const closeModal = useCallback(
    (event) => {
      if (event.key === 'Escape') {
        onClose();
      }
      if (event.key === 'Enter') {
        handleContinue();
      }
    },
    [onClose, handleContinue],
  );

  useEffect(() => {
    if (visible) {
      window.addEventListener('keydown', closeModal);
    } else {
      window.removeEventListener('keydown', closeModal);
    }
    return () => {
      window.removeEventListener('keydown', closeModal);
    };
  }, [visible, closeModal]);
  return (
    <div className="modal" style={{ display: visible ? 'block' : 'none' }}>
      <div className="modal-content">
        <button
          className="close"
          type="button"
          aria-label="Close modal"
          onClick={onClose}
        >
          <svg
            width="11"
            height="11"
            viewBox="0 0 11 11"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M6.8061 5.50023L10.8919 1.41468C11.037 1.26959 11.037 1.03481 10.8919 0.889774L10.1111 0.108628C10.0415 0.0391301 9.94713 0 9.84863 0C9.7501 0 9.65572 0.0391301 9.58618 0.108628L5.50036 4.19449L1.41455 0.108628C1.345 0.0391301 1.25063 0 1.1521 0C1.05359 0 0.959195 0.0391301 0.889673 0.108628L0.108813 0.889774C-0.0362711 1.03481 -0.0362711 1.26959 0.108813 1.41468L4.19463 5.50025L0.109219 9.58532C-0.0357937 9.73041 -0.0357937 9.96519 0.109219 10.1102L0.890126 10.8914C0.959625 10.9609 1.05402 11 1.15255 11C1.25108 11 1.34546 10.9609 1.41503 10.8914L5.50034 6.80599L9.58568 10.8914C9.65522 10.9609 9.7496 11 9.84813 11C9.94668 11 10.0411 10.9609 10.1106 10.8914L10.8915 10.1102C11.0365 9.96519 11.0365 9.73041 10.8915 9.58532L6.8061 5.50023Z"
              fill="#BEBEBE"
            />
          </svg>
        </button>

        <div className="modal-header">WARNING!</div>
        <div className="modal-sub-header">
          SHUTDOWN WHILE THE DRONE IS MIDAIR MAY HAVE CATASTROPHIC CONSEQUENCES
        </div>
        <div className="modal-sub-header">PLEASE CONTINUE WITH CAUTION</div>
        <div className="modal-description">
          This is intended for post-flight or emergency use.
          <br />
          CONTINUE TO DISARM?
        </div>
        <div>
          <button
            className="modal-button"
            type="button"
            onClick={handleContinue}
          >
            CONTINUE
          </button>
        </div>
      </div>
    </div>
  );
}
Modal.propTypes = {
  ip: PropTypes.string.isRequired,
  visible: PropTypes.bool.isRequired,
  setVisible: PropTypes.func.isRequired,
  setLoggedIn: PropTypes.func.isRequired,
  onClose: PropTypes.func.isRequired,
};
export default Modal;
