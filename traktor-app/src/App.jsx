import { BrowserRouter as Router, Routes, Route, Link } from 'react-router-dom';
import ChartPage from './pages/ChartPage';
import MotorControl from './pages/MotorControl'; // Halaman kontrol motor yang sudah ada
import AutoPage from './pages/AutoPage';
import MapPage from './pages/MapPage'

function App() {
  return (
    <Router>
      <nav className="flex bg-gray-800 p-4 justify-center">
        <ul className="flex space-x-4">
          <li>
            <Link to="/" className="text-white hover:text-gray-300">Beranda</Link>
          </li>
          <li>
            <Link to="/motor-control" className="text-white hover:text-gray-300">Kontrol Manual</Link>
          </li>
          <li>
            <Link to="/auto" className="text-white hover:text-gray-300">Kontrol Otomatis</Link>
          </li>
          <li>
            <Link to="/chart" className="text-white hover:text-gray-300">Grafik IMU-GPS</Link>
          </li>
        </ul>
      </nav>

      <Routes>
        <Route path="/" />
        <Route path="/motor-control" element={<MotorControl />} />
        <Route path="/chart" element={<ChartPage />} />
        <Route path="/auto" element={<AutoPage />} />
        <Route path="/map" element={<MapPage/>} />
      </Routes>
    </Router>
  );
}

export default App;