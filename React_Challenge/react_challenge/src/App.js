import React, { useState, useEffect } from 'react';
import './App.css';

function App() {
  const [counter, setCounter] = useState(1);
  const [userData, setUserData] = useState(null);

  useEffect(() => {
    fetch(`https://jsonplaceholder.typicode.com/users/${counter}`)
      .then(response => response.json())
      .then(data => setUserData(data))
      .catch(error => {
        console.error('Error fetching user data:', error);
        setUserData(null);
      });
  }, [counter]);

  const incrementCounter = () => {
    setCounter((prevCounter) => (prevCounter < 10 ? prevCounter + 1 : prevCounter));
  };

  const decrementCounter = () => {
    setCounter((prevCounter) => (prevCounter > 1 ? prevCounter - 1 : prevCounter));
  };

  return (
    <div className="App">
      <header className="App-header">
        <h1>User Data Viewer</h1>
        <div>
          <button onClick={decrementCounter} disabled={counter === 1}>
            Previous User
          </button>
          <span style={{ margin: '0 20px' }}>User ID: {counter}</span>
          <button onClick={incrementCounter} disabled={counter === 10}>
            Next User
          </button>
        </div>
        {userData ? (
          <div className="User-data">
            <h2>{userData.name}</h2>
            <p>
              <strong>Email:</strong> {userData.email}
            </p>
            <p>
              <strong>Phone:</strong> {userData.phone}
            </p>
            <p>
              <strong>Website:</strong> {userData.website}
            </p>
          </div>
        ) : (
          <p>Loading user data...</p>
        )}
      </header>
    </div>
  );
}

export default App;
