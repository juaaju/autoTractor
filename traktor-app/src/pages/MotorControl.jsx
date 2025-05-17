import { useState } from 'react';

const RemoteControl = () => {
  const [status, setStatus] = useState('');
  const [pressing, setPressing] = useState(null);

  const turnRight = async () => {
    setPressing('turnRight');
    try {
      const response = await fetch('http://ubuntu.local:5000/turn_right');
      const data = await response.json();
      setStatus(data.status);
    } catch (error) {
      console.error('Error turning right:', error);
    } finally {
      setTimeout(() => setPressing(null), 100);
    }
  };

  const turnLeft = async () => {
    setPressing('turnLeft');
    try {
      const response = await fetch('http://ubuntu.local:5000/turn_left');
      const data = await response.json();
      setStatus(data.status);
    } catch (error) {
      console.error('Error turning left:', error);
    } finally {
      setTimeout(() => setPressing(null), 100);
    }
  };

  const cwRight = async () => {
    setPressing('cwRight');
    try {
      const response = await fetch('http://ubuntu.local:5000/cw_right');
      const data = await response.json();
      setStatus(data.status);
    } catch (error) {
      console.error('Error turning right:', error);
    } finally {
      setTimeout(() => setPressing(null), 100);
    }
  };

  const ccwRight = async () => {
    setPressing('ccwRight');
    try {
      const response = await fetch('http://ubuntu.local:5000/ccw_right');
      const data = await response.json();
      setStatus(data.status);
    } catch (error) {
      console.error('Error turning left:', error);
    } finally {
      setTimeout(() => setPressing(null), 100);
    }
  };

  const cwLeft = async () => {
    setPressing('cwLeft');
    try {
      const response = await fetch('http://ubuntu.local:5000/cw_left');
      const data = await response.json();
      setStatus(data.status);
    } catch (error) {
      console.error('Error turning right:', error);
    } finally {
      setTimeout(() => setPressing(null), 100);
    }
  };

  const ccwLeft = async () => {
    setPressing('ccwLeft');
    try {
      const response = await fetch('http://ubuntu.local:5000/ccw_left');
      const data = await response.json();
      setStatus(data.status);
    } catch (error) {
      console.error('Error turning left:', error);
    } finally {
      setTimeout(() => setPressing(null), 100);
    }
  };

  const motorOff = async () => {
    setPressing('motorOff');
    try {
      const response = await fetch('http://ubuntu.local:5000/motor_off');
      const data = await response.json();
      setStatus(data.status);
    } catch (error) {
      console.error('Error turning off:', error);
    } finally {
      setTimeout(() => setPressing(null), 100);
    }
  };

  return (
    <div className="bg-gray-900 text-white p-6 max-w-lg mx-auto rounded-3xl shadow-2xl border-2 border-gray-800">
      {/* Display Screen */}
      <div className="bg-gray-800 mb-6 p-4 rounded-lg border border-gray-700 flex justify-between items-center">
        <div className="flex items-center">
          <div className="h-2 w-2 rounded-full bg-green-500 mr-2 animate-pulse"></div>
          <span className="text-sm text-gray-400">STATUS</span>
        </div>
        <div className="bg-gray-700 px-4 py-2 rounded-md text-green-400 font-mono tracking-wider">
          {status || "Siap"}
        </div>
      </div>

      {/* Main Controls */}
      <div className="mb-6">
        <h1 className="text-center mb-4 text-xl font-extrabold leading-none tracking-tight text-gray-400">Kontrol Manual:</h1>
        <div className="flex justify-center gap-4">
          <button 
            onClick={turnLeft}
            className={`
              m-2 rounded-full w-16 h-16 flex items-center justify-center font-bold text-white transition-all
              ${pressing === 'turnLeft' ? 'bg-green-600 translate-y-1 shadow-none' : 'bg-green-500 shadow-lg border-b-4 border-green-700 hover:bg-green-400 hover:border-green-500'}
            `}
          >
            &lt;
          </button>
          <button 
            onClick={motorOff}
            className={`
              m-2 rounded-full w-16 h-16 flex items-center justify-center font-bold text-white transition-all
              ${pressing === 'motorOff' ? 'bg-red-600 translate-y-1 shadow-none' : 'bg-red-500 shadow-lg border-b-4 border-red-700 hover:bg-red-400 hover:border-red-500'}
            `}
          >
            OFF
          </button>
          <button 
            onClick={turnRight}
            className={`
              m-2 rounded-full w-16 h-16 flex items-center justify-center font-bold text-white transition-all
              ${pressing === 'turnRight' ? 'bg-blue-600 translate-y-1 shadow-none' : 'bg-blue-500 shadow-lg border-b-4 border-blue-700 hover:bg-blue-400 hover:border-blue-500'}
            `}
          >
            &gt;
          </button>
        </div>
      </div>

      {/* Testing Section */}
      <div>
        <h1 className="text-center mb-4 text-xl font-extrabold leading-none tracking-tight text-gray-400">Testing Motor:</h1>
        <div className="flex flex-wrap justify-center gap-2">
          <button 
            onClick={cwLeft}
            className={`
              m-2 rounded-full w-16 h-16 flex items-center justify-center font-bold text-white text-sm transition-all
              ${pressing === 'cwLeft' ? 'bg-green-600 translate-y-1 shadow-none' : 'bg-green-500 shadow-lg border-b-4 border-green-700 hover:bg-green-400 hover:border-green-500'}
            `}
          >
            CW L
          </button>
          <button 
            onClick={ccwLeft}
            className={`
              m-2 rounded-full w-16 h-16 flex items-center justify-center font-bold text-white text-sm transition-all
              ${pressing === 'ccwLeft' ? 'bg-green-600 translate-y-1 shadow-none' : 'bg-green-500 shadow-lg border-b-4 border-green-700 hover:bg-green-400 hover:border-green-500'}
            `}
          >
            CCW L
          </button>
          <button 
            onClick={cwRight}
            className={`
              m-2 rounded-full w-16 h-16 flex items-center justify-center font-bold text-white text-sm transition-all
              ${pressing === 'cwRight' ? 'bg-blue-600 translate-y-1 shadow-none' : 'bg-blue-500 shadow-lg border-b-4 border-blue-700 hover:bg-blue-400 hover:border-blue-500'}
            `}
          >
            CW R
          </button>
          <button 
            onClick={ccwRight}
            className={`
              m-2 rounded-full w-16 h-16 flex items-center justify-center font-bold text-white text-sm transition-all
              ${pressing === 'ccwRight' ? 'bg-blue-600 translate-y-1 shadow-none' : 'bg-blue-500 shadow-lg border-b-4 border-blue-700 hover:bg-blue-400 hover:border-blue-500'}
            `}
          >
            CCW R
          </button>
        </div>
      </div>

      {/* Power & Brand */}
      <div className="mt-8 flex justify-between items-center">
        <div className="text-xs text-gray-500 font-semibold">v1.0</div>
        <div className="text-center">
          <div className="w-6 h-6 mx-auto bg-red-500 rounded-full border-2 border-red-700 flex items-center justify-center mb-1">
            <div className="w-4 h-4 bg-red-600 rounded-full"></div>
          </div>
          <div className="text-xs text-gray-500">POWER</div>
        </div>
        <div className="text-xs text-gray-500 font-semibold">2025</div>
      </div>
    </div>
  );
};

export default RemoteControl;