import { useState } from 'react';
import axios from 'axios';


function MotorControl() {
  const [status, setStatus] = useState('');

  const turnRight = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/turn_right');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning right:', error);
    }
  };

  const turnLeft = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/turn_left');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning left:', error);
    }
  };

  const cwRight = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/cw_right');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning right:', error);
    }
  };

  const ccwRight = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/ccw_right');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning left:', error);
    }
  };

  const cwLeft = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/cw_left');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning right:', error);
    }
  };

  const ccwLeft = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/ccw_left');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning left:', error);
    }
  };

  const motorOff = async () => {
    try {
      const response = await axios.get('http://ubuntu.local:5000/motor_off');
      setStatus(response.data.status);
    } catch (error) {
      console.error('Error turning off:', error);
    }
  };

  return (

    <div className="h-[calc(100vh-2rem)] flex flex-col justify-center items-center">
      <h1 className=" mb-4 text-3xl font-extrabold leading-none tracking-tight text-gray-800">Kontrol Manual:</h1>
      <div className="flex mb-8">
        <button onClick={turnRight} className="m-2 bg-blue-500 hover:bg-blue-400 text-white font-bold w-16 h-16 border-b-4 border-blue-700 hover:border-blue-500 rounded-full">Right</button>
        <button onClick={turnLeft} className="m-2 bg-green-500 hover:bg-green-400 text-white font-bold w-16 h-16 border-b-4 border-green-700 hover:border-green-500 rounded-full">Left</button>
        <button onClick={motorOff} className="m-2 bg-red-500 hover:bg-red-400 text-white font-bold w-16 h-16 border-b-4 border-red-700 hover:border-red-500 rounded-full">Off</button>
      </div>
      <div><h1 className=" mb-4 text-3xl font-extrabold leading-none tracking-tight text-gray-800">Testing Motor:</h1></div>
      <div className="flex">
        <button onClick={cwRight} className="m-2 bg-blue-500 hover:bg-blue-400 text-white font-bold w-16 h-16 border-b-4 border-blue-700 hover:border-blue-500 rounded-full">CW R</button>
        <button onClick={ccwLeft} className="m-2 bg-blue-500 hover:bg-blue-400 text-white font-bold w-16 h-16 border-b-4 border-blue-700 hover:border-blue-500 rounded-full">CCW R</button>
        <button onClick={cwLeft} className="m-2 bg-green-500 hover:bg-green-400 text-white font-bold w-16 h-16 border-b-4 border-green-700 hover:border-green-500 rounded-full">CW L</button>
        <button onClick={ccwLeft} className="m-2 bg-green-500 hover:bg-green-400 text-white font-bold w-16 h-16 border-b-4 border-green-700 hover:border-green-500 rounded-full">CCW L</button>

      </div>
      <p>{status}</p>
    </div>
  );
}

export default MotorControl;
