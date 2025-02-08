import { useState } from 'react';
import axios from 'axios';

function ControlButton({ onClick, label, color, size }) {
  return (
    <button
      onClick={onClick}
      className={`m-2 bg-${color}-500 hover:bg-${color}-400 text-white font-bold w-${size} h-${size} border-b-4 border-${color}-700 hover:border-${color}-500 rounded-full`}
    >
      {label}
    </button>
  );
}

function App() {
  const [status, setStatus] = useState('');

  const turnRight = async () => {
    try {
      const response = await axios.get('http://localhost:5000/turn_right');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning on the LED:', error);
    }
  };

  const turnLeft = async () => {
    try {
      const response = await axios.get('http://localhost:5000/turn_left');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning off the LED:', error);
    }
  };

  const motorOff = async () => {
    try {
      const response = await axios.get('http://localhost:5000/motor_off');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning off the LED:', error);
    }
  };

  return (
    <div className="App flex flex-col justify-center items-center min-h-screen">
      <h1 className="mb-4 text-4xl font-extrabold leading-none tracking-tight text-gray-900 md:text-5xl lg:text-6xl dark:text-white">
        Kontrol Traktor
      </h1>
      <div className="flex">
        <ControlButton onClick={turnRight} label="Right" color="blue" size="16" />
        <ControlButton onClick={turnLeft} label="Left" color="blue" size="16" />
        <ControlButton onClick={motorOff} label="Off" color="red" size="16" />
      </div>
      <p>{status}</p>
    </div>
  );
}

export default App;
