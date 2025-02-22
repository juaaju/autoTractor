// src/App.jsx
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import ChartPage from './pages/ChartPage';
import MotorControl from './pages/MotorControl';
import AutoPage from './pages/AutoPage';
import MapPage from './pages/MapPage';
import Home from './pages/Home';
import SideBarLayout from './components/SideBar';
import { useState, useEffect } from 'react';

function App() {
  const [isCollapsed, setIsCollapsed] = useState(false);

  useEffect(() => {
    const checkMobile = () => {
      setIsCollapsed(window.innerWidth < 768);
    };
    
    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

  return (
    <Router>
      <div className="flex min-h-screen bg-gray-100">
        <SideBarLayout isCollapsed={isCollapsed} setIsCollapsed={setIsCollapsed} />
        <div 
          className={`flex-1 ${
            isCollapsed ? 'pl-16' : 'pl-64'
          } transition-all duration-300 min-h-screen w-full`}
        >
          <main className="p-4">
            <Routes>
              <Route path="/" element={<Home/>}/>
              <Route path="/motor-control" element={<MotorControl />} />
              <Route path="/chart" element={<ChartPage />} />
              <Route path="/auto" element={<AutoPage />} />
              <Route path="/map" element={<MapPage />} />
            </Routes>
          </main>
        </div>
      </div>
    </Router>
  );
}

export default App;