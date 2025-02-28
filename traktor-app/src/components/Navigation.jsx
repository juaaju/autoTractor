// src/components/Navigation.jsx
import React from 'react';
import { Link, useLocation } from 'react-router-dom';
import { Home, GitFork, PlayCircle, LineChart, Map, ChevronLeft, ChevronRight } from 'lucide-react';

// Navbar komponen untuk mobile
const MobileNavbar = () => {
  const location = useLocation();

  return (
    <nav className="fixed bottom-0 left-0 w-full bg-gray-800 md:hidden z-[9999]">
      <div className="flex justify-around items-center h-16">
        <NavLink to="/" icon={<Home size={24} />} isActive={location.pathname === '/'} />
        <NavLink 
          to="/motor-control" 
          icon={<GitFork size={24} />} 
          isActive={location.pathname === '/motor-control'} 
        />
        <NavLink 
          to="/auto" 
          icon={<PlayCircle size={24} />} 
          isActive={location.pathname === '/auto'} 
        />
        <NavLink 
          to="/chart" 
          icon={<LineChart size={24} />} 
          isActive={location.pathname === '/chart'} 
        />
        <NavLink 
          to="/map" 
          icon={<Map size={24} />} 
          isActive={location.pathname === '/map'} 
        />
      </div>
    </nav>
  );
};

// Sidebar komponen untuk desktop
const DesktopSidebar = ({ isCollapsed, setIsCollapsed }) => {
  const location = useLocation();

  const toggleSidebar = () => {
    setIsCollapsed(!isCollapsed);
  };

  return (
    <aside 
      className={`hidden md:block bg-gray-800 fixed left-0 top-0 h-full z-20
        ${isCollapsed ? 'w-16' : 'w-64'} transition-all duration-300`}
    >
      <div className="flex flex-col h-full relative">
        {/* Toggle Button */}
        <button
          onClick={toggleSidebar}
          className="absolute -right-3 top-6 bg-gray-800 rounded-full p-1 text-gray-300 hover:text-white z-30"
        >
          {isCollapsed ? <ChevronRight size={20} /> : <ChevronLeft size={20} />}
        </button>

        {/* Logo/Header Area */}
        <div className="p-4 border-b border-gray-700">
          <h1 className={`text-white font-bold ${isCollapsed ? 'text-sm text-center' : 'text-xl'}`}>
            {isCollapsed ? 'CP' : 'Control Panel'}
          </h1>
        </div>

        {/* Navigation Links */}
        <nav className="flex-1 pt-4">
          <ul className="space-y-2">
            <SidebarLink 
              to="/" 
              icon={<Home />} 
              text="Beranda" 
              isCollapsed={isCollapsed}
              isActive={location.pathname === '/'} 
            />
            <SidebarLink 
              to="/motor-control" 
              icon={<GitFork />} 
              text="Kontrol Manual" 
              isCollapsed={isCollapsed}
              isActive={location.pathname === '/motor-control'} 
            />
            <SidebarLink 
              to="/auto" 
              icon={<PlayCircle />} 
              text="Kontrol Otomatis" 
              isCollapsed={isCollapsed}
              isActive={location.pathname === '/auto'} 
            />
            <SidebarLink 
              to="/chart" 
              icon={<LineChart />} 
              text="Grafik IMU-GPS" 
              isCollapsed={isCollapsed}
              isActive={location.pathname === '/chart'} 
            />
            <SidebarLink 
              to="/map" 
              icon={<Map />} 
              text="Peta" 
              isCollapsed={isCollapsed}
              isActive={location.pathname === '/map'} 
            />
          </ul>
        </nav>
      </div>
    </aside>
  );
};

// Komponen untuk link di navbar mobile
const NavLink = ({ to, icon, isActive }) => (
  <Link
    to={to}
    className={`p-2 rounded-lg ${
      isActive 
        ? 'text-white bg-gray-700' 
        : 'text-gray-400 hover:text-white hover:bg-gray-700'
    }`}
  >
    {icon}
  </Link>
);

// Komponen untuk link di sidebar desktop
const SidebarLink = ({ to, icon, text, isCollapsed, isActive }) => (
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

// Komponen utama yang menggabungkan Navbar dan Sidebar
const Navigation = ({ isCollapsed, setIsCollapsed }) => {
  return (
    <>
      <MobileNavbar />
      <DesktopSidebar isCollapsed={isCollapsed} setIsCollapsed={setIsCollapsed} />
    </>
  );
};

export default Navigation;