// src/components/SideBar.jsx
import React, { useState, useEffect } from 'react';
import { Link, useLocation } from 'react-router-dom';
import { Home, GitFork, PlayCircle, LineChart, Map, ChevronLeft, ChevronRight } from 'lucide-react';

const SidebarLayout = ({ isCollapsed, setIsCollapsed }) => {
  const [isMobile, setIsMobile] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const checkMobile = () => {
      const isMobileView = window.innerWidth < 768;
      setIsMobile(isMobileView);
      if (isMobileView) {
        setIsCollapsed(true);
      }
    };

    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, [setIsCollapsed]);

  const toggleSidebar = () => {
    if (!isMobile) {
      setIsCollapsed(!isCollapsed);
    }
  };

  return (
    <aside 
      className={`bg-gray-800 fixed left-0 top-0 h-full z-20
        ${isCollapsed ? 'w-16' : 'w-64'} transition-all duration-300`}
    >
      <div className="flex flex-col h-full">
        {/* Toggle Button - Hidden on Mobile */}
        {!isMobile && (
          <button
            onClick={toggleSidebar}
            className="absolute -right-3 top-6 bg-gray-800 rounded-full p-1 text-gray-300 hover:text-white z-30"
          >
            {isCollapsed ? <ChevronRight size={20} /> : <ChevronLeft size={20} />}
          </button>
        )}

        {/* Logo/Header Area */}
        <div className="p-4 border-b border-gray-700">
          <h1 className={`text-white font-bold ${isCollapsed ? 'text-sm text-center' : 'text-xl'}`}>
            {isCollapsed ? 'CP' : 'Control Panel'}
          </h1>
        </div>

        {/* Navigation Links */}
        <nav className="flex-1 pt-4 overflow-y-auto">
          <ul className="space-y-2">
            <NavItem to="/" icon={<Home />} text="Beranda" isCollapsed={isCollapsed} isActive={location.pathname === '/'} />
            <NavItem 
              to="/motor-control" 
              icon={<GitFork />} 
              text="Kontrol Manual" 
              isCollapsed={isCollapsed} 
              isActive={location.pathname === '/motor-control'} 
            />
            <NavItem 
              to="/auto" 
              icon={<PlayCircle />} 
              text="Kontrol Otomatis" 
              isCollapsed={isCollapsed} 
              isActive={location.pathname === '/auto'} 
            />
            <NavItem 
              to="/chart" 
              icon={<LineChart />} 
              text="Grafik IMU-GPS" 
              isCollapsed={isCollapsed} 
              isActive={location.pathname === '/chart'} 
            />
            <NavItem 
              to="/map" 
              icon={<Map />} 
              text="Peta Realtime" 
              isCollapsed={isCollapsed} 
              isActive={location.pathname === '/map'} 
            />
          </ul>
        </nav>
      </div>
    </aside>
  );
};

const NavItem = ({ to, icon, text, isCollapsed, isActive }) => (
  <li>
    <Link
      to={to}
      className={`flex items-center text-gray-300 hover:bg-gray-700 px-4 py-3 transition-colors
        ${isActive ? 'bg-gray-700 text-white' : ''}`}
    >
      <div className={`${isCollapsed ? 'mx-auto' : 'mr-3'}`}>
        {React.cloneElement(icon, { size: 20 })}
      </div>
      {!isCollapsed && <span className="whitespace-nowrap">{text}</span>}
    </Link>
  </li>
);

export default SidebarLayout;