import PropTypes from 'prop-types';
import React, { useEffect, useState } from 'react';
import axios from 'axios';

import ManageUsers from './ManageUsers';

export default function AdminSection({ serverIp, user }) {
  /* OPTIONS: "Manage Users", "Create New User" */
  const [body, setBody] = useState('Manage Users');
  const [userList, setUserList] = useState();
  /* */
  const [name, setName] = useState('');
  const [pin, setPin] = useState('');
  const [role, setRole] = useState('Operator');

  useEffect(() => {
    let isMounted = true;
    // Define the function to get the user list
    const getUserList = async () => {
      try {
        // Make the GET request using axios
        const response = await axios.get(`http://${serverIp}:3080/users`, {
          headers: {
            'Content-Type': 'application/json',
            'jwt-token': user.token,
            role: user.role,
          },
        });

        // Check if the component is still mounted
        if (isMounted) {
          // Set the user list with the response data
          setUserList(response.data.users);
        }
      } catch (error) {
        // Handle error if the request fails
        console.error('Error fetching user list:', error);
      }
    };

    // If the user has admin privileges
    if (user.role === 'Administrator') {
      // Call the function or integrate it as needed in your component
      getUserList();
    }
    return () => {
      isMounted = false;
    };
  }, [serverIp, user.role, user.token]);

  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.key === 'Escape') {
        setBody('Manage Users');
      }
    };

    window.addEventListener('keydown', handleKeyDown);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  const handleRemoveUser = async (userId) => {
    try {
      // Create request payload
      const request = {
        action: 'remove',
        user: { id: userId },
      };

      // Send POST request to remove user
      const response = await axios.post(
        `http://${serverIp}:3080/users`,
        request,
        {
          headers: {
            'jwt-token': user.token,
          },
        },
      );

      // Check if the response status is 200 (OK)
      if (response.status === 200) {
        console.log('User removed successfully');
        // Update the user list
        setUserList(response.data.users);
      }
    } catch (error) {
      console.error('Error removing user:', error);
    }
  };
  const handleCreateUser = async () => {
    try {
      // Hash the pin and create the request object
      const req = {
        action: 'add',
        user: {
          name,
          role,
          pin,
        },
      };

      // Send the new user data to the server
      const response = await axios.post(`http://${serverIp}:3080/users`, req, {
        headers: { 'jwt-token': user.token },
      });

      if (response.status === 200) {
        console.log('User created successfully');
        // Update the user list and change the body
        setUserList(response.data.users);
        setBody('Manage Users');
      }
    } catch (error) {
      console.error('Error creating user:', error);
    }
  };

  const handleEditUser = (username, userrole) => {
    setBody('Edit User');
    setName(username);
    setRole(userrole);
  };
  const handleSubmitEdit = async (userId, newPin, newRole) => {
    try {
      // Create the request object
      const req = {
        action: 'edit',
        user: {
          id: userId,
          pin: newPin,
          role: newRole,
        },
      };

      // Send the updated user data to the server
      const response = await axios.post(`http://${serverIp}:3080/users`, req, {
        headers: { 'jwt-token': user.token },
      });

      if (response.status === 200) {
        console.log('Pin edited successfully');
        // Change the body
        setBody('Manage Users');
      }
    } catch (error) {
      // console.error('Error changing pin:', error);
    }
  };

  // console.log(body)
  return (
    <div className="admin-main">
      <div className="header">Administrator Settings</div>
      <div className="line" />
      {body === 'Manage Users' && (
        <div className="manage-users-body">
          {userList &&
            userList.map((user) => (
              <div key={user.id} className="user-card">
                <button
                  type="button"
                  className="remove-user-button"
                  onClick={() => handleRemoveUser(user.id)}
                >
                  X
                </button>
                <div className="user-info">
                  <div className="header">{user.name}</div>
                  <div className="sub-header">
                    <strong>Role:</strong> {user.role}
                  </div>
                </div>
                <div className="user-actions">
                  <button
                    type="button"
                    onClick={() =>
                      handleEditUser(user.name, user.role, user.id)
                    }
                  >
                    Edit User
                  </button>
                </div>
              </div>
            ))}
          <button
            type="button"
            className="create-card"
            onClick={() => setBody('Create New User')}
          >
            Create New User
          </button>
        </div>
      )}
      <ManageUsers
        type={body}
        name={name}
        role={role}
        handleCreateUser={handleCreateUser}
        handleSubmitEdit={handleSubmitEdit}
        setName={setName}
        setPin={setPin}
        setRole={setRole}
      />
    </div>
  );
}
AdminSection.propTypes = {
  serverIp: PropTypes.string.isRequired,
  user: PropTypes.string.isRequired,
};
