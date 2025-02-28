import { useState } from 'react';
import axios from 'axios';


function MotorControl() {
  const [status, setStatus] = useState('');

  const turnRight = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/turn_right');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning on the LED:', error);
    }
  };

  const turnLeft = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/turn_left');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning off the LED:', error);
    }
  };

  const motorOff = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/motor_off');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning off the LED:', error);
    }
  };

  return (

    <div className="h-[calc(100vh-2rem)] flex flex-col justify-center items-center">
      <h1 className=" mb-4 text-4xl font-extrabold leading-none tracking-tight text-gray-800 md:text-4xl lg:text-5xl">Kontrol Manual</h1>
      <div className="flex">
        <button onClick={turnRight} className="m-2 bg-blue-500 hover:bg-blue-400 text-white font-bold w-16 h-16 border-b-4 border-blue-700 hover:border-blue-500 rounded-full">Right</button>
        <button onClick={turnLeft} className="m-2 bg-blue-500 hover:bg-blue-400 text-white font-bold w-16 h-16 border-b-4 border-blue-700 hover:border-blue-500 rounded-full">Left</button>
        <button onClick={motorOff} className="m-2 bg-red-500 hover:bg-red-400 text-white font-bold w-16 h-16 border-b-4 border-red-700 hover:border-red-500 rounded-full">Off</button>
      </div>
      <p>{status}</p>
    </div>
  );
}

export default MotorControl;
