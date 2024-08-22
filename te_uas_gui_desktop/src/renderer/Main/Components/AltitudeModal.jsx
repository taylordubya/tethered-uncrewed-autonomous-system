/* Styling */
import './AltitudeModal.css';

/* Default */
import PropTypes from 'prop-types';
import { useState, useEffect, useRef } from 'react';

/*  */
function AltitudeModal({
  isVisible,
  setIsVisible,
  customAltitude,
  handleAltitudeChange,
  handleLaunch,
}) {
  const [modalPosition, setModalPosition] = useState({ x: 500, y: 200 });
  const modalRef = useRef(null);
  const inputRef = useRef(null);

  /* Select the input when the modal is loaded */
  useEffect(() => {
    if (isVisible && inputRef.current) {
      inputRef.current.select();
    }
  }, [isVisible]);

  /* Handle modal drag and drop */
  const handleMouseDown = (e) => {
    const startX = e.clientX;
    const startY = e.clientY;
    const modal = modalRef.current;
    const rect = modal.getBoundingClientRect();
    const offsetX = startX - rect.left;
    const offsetY = startY - rect.top + 65;

    const handleMouseMove = (moveEvent) => {
      const newX = moveEvent.clientX - offsetX;
      const newY = moveEvent.clientY - offsetY;
      setModalPosition({ x: newX, y: newY });
    };

    const handleMouseUp = () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };

    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);
  };
  return (
    <div className="AltitudeModal">
      {isVisible && (
        <div
          className="altitude-modal"
          style={{ top: `${modalPosition.y}px`, left: `${modalPosition.x}px` }}
          ref={modalRef}
        >
          <div className="altitude-modal-header" onMouseDown={handleMouseDown}>
            <span>Launch to Altitude (m)</span>
            <button
              type="button"
              className="exit"
              onClick={() => setIsVisible(false)}
            >
              X
            </button>
          </div>
          <div className="altitude-modal-content">
            <input
              ref={inputRef}
              inputMode="numeric"
              maxLength={3}
              max={100}
              value={customAltitude || ''}
              onChange={(e) => handleAltitudeChange(e)}
              // onKeyDown={(e) => e.key === "Enter" && confirmModal()}
            />
          </div>
          <button
            className="altitude-modal-launch"
            type="button"
            onClick={() => {
              if (customAltitude > 0) {
                setIsVisible(false);
                handleLaunch();
              } else {
                /* Empty */
              }
            }}
          >
            GO
          </button>
        </div>
      )}
    </div>
  );
}
AltitudeModal.propTypes = {
  isVisible: PropTypes.bool.isRequired,
  setIsVisible: PropTypes.func.isRequired,
  customAltitude: PropTypes.string.isRequired,
  handleAltitudeChange: PropTypes.func.isRequired,
  handleLaunch: PropTypes.func.isRequired,
};
export default AltitudeModal;
