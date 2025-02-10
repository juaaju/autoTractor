import { useState } from 'react';
import axios from 'axios';


function MotorControl() {
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
      <h1 className=" mb-4 text-4xl font-extrabold leading-none tracking-tight text-gray-900 md:text-5xl lg:text-6xl dark:text-white">Kontrol Motor</h1>
      <div className="flex">
        <button onClick={turnRight} className="m-2 bg-blue-500 hover:bg-blue-400 text-white font-bold w-16 h-16 border-b-4 border-blue-700 hover:border-blue-500 rounded-full">Right</button>
        <button onClick={turnLeft} className="m-2 bg-blue-500 hover:bg-blue-400 text-white font-bold w-16 h-16 border-b-4 border-blue-700 hover:border-blue-500 rounded-full">Left</button>
        <button onClick={motorOff} className="m-2 bg-red-500 hover:bg-red-400 text-white font-bold w-16 h-16 border-b-4 border-red-700 hover:border-red-500 rounded-full">Off</button>
      </div>
      <p class="">{status}</p>
    </div>
  );
}

export default MotorControl;
