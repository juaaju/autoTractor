// src/App.jsx
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { useState, useEffect } from 'react';
import Navigation from './components/Navigation';
import Home from './pages/Home';
import ChartPage from './pages/ChartPage';
import MotorControl from './pages/MotorControl';
import AutoPage from './pages/AutoPage';
import MapPage from './pages/MapPage';

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
      <div className="flex min-h-screen">
        <Navigation isCollapsed={isCollapsed} setIsCollapsed={setIsCollapsed} />
        
        <div 
          className={`flex-1 pb-20 md:pb-0 transition-all duration-300
            ${isCollapsed ? 'md:ml-16' : 'md:ml-64'}`}
        >
          <main className="p-4">
            <Routes>
              <Route path="/" element={<Home />} />
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