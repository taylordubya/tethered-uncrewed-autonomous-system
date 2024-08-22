import './Modules.css';
/* Packages */
import { useState } from 'react';

function Camera({ ip }) {
  const [loading, setLoading] = useState(true);
  return (
    <div className="Camera">
      <iframe
        className="video-stream"
        src={`http://${ip}:8889/cam`}
        width="1280"
        height="720"
      />
    </div>
  );
}
export default Camera;
