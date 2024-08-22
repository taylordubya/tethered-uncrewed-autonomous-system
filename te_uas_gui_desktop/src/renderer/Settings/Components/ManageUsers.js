import PropTypes from 'prop-types';
import React, { useState, useEffect } from 'react';
import '../Settings.css';

function ManageUsers({
  type,
  handleCreateUser,
  handleSubmitEdit,
  name,
  setName,
  setPin,
  role,
  setRole,
}) {
  const [description, setDescription] = useState('');

  const handleFocus = (desc) => {
    setDescription(desc);
  };

  const handleBlur = () => {
    setDescription('');
  };

  const nameDescription = (
    <div className="create-description">
      <div>Enter the full name of the user.</div>
    </div>
  );

  const pinDescription = (
    <div className="create-description">
      <div>Create a four-digit login pin for the user.</div>
    </div>
  );

  const roleDescription = (
    <div className="create-description">
      <div>
        Select the role of the user. Regular users should be set as an operator.
      </div>
      <div className="create-desc-warning">
        <div className="warning-label">
          <i>WARNING</i>
        </div>
        <div>
          <i>
            Administrator users can manage other users, including
            creating/deleting accounts and changing pins.
          </i>
        </div>
      </div>
    </div>
  );

  if (type !== 'Create New User' && type !== 'Edit User') {
    return <div />;
  }

  return (
    <div className="create-user-body">
      <div className="setting-description">
        <div className="header">Setting Description</div>
        <div className="line" />
        {description}
      </div>
      <div className="user-settings">
        <div className="header">
          {type === 'Create New User' ? 'Create New User' : `Edit: ${name}`}
        </div>
        <div className="line" />

        {type === 'Create New User' && (
          <div className="create-name">
            <div className="m1-header">Name</div>
            <input
              onChange={(e) => setName(e.target.value)}
              onFocus={() => handleFocus(nameDescription)}
              onBlur={handleBlur}
            />
          </div>
        )}

        <div className="create-pin">
          <div className="m1-header">Pin</div>
          <input
            className="change-pin"
            maxLength={4}
            type="password"
            pattern="\d*"
            onChange={(e) => setPin(e.target.value)}
            onFocus={() => handleFocus(pinDescription)}
            onBlur={handleBlur}
          />
        </div>

        <div className="create-role">
          <div className="m2-header">Role</div>
          <select
            value={role}
            onChange={(e) => setRole(e.target.value)}
            onFocus={() => handleFocus(roleDescription)}
            onBlur={handleBlur}
          >
            <option value="Operator">Operator</option>
            <option value="Administrator">Administrator</option>
          </select>
        </div>

        <button
          type="button"
          className="create-button"
          onClick={
            type === 'Create New User' ? handleCreateUser : handleSubmitEdit
          }
        >
          {type === 'Create New User' ? 'Create' : 'Edit'}
        </button>
      </div>
    </div>
  );
}

ManageUsers.propTypes = {
  type: PropTypes.string.isRequired,
  handleCreateUser: PropTypes.func.isRequired,
  handleSubmitEdit: PropTypes.func.isRequired,
  name: PropTypes.string.isRequired,
  setName: PropTypes.func.isRequired,
  pin: PropTypes.string.isRequired,
  setPin: PropTypes.func.isRequired,
  role: PropTypes.string.isRequired,
  setRole: PropTypes.func.isRequired,
};
export default ManageUsers;
