import React, { useState } from "react";

const TractorForm = ({ onSubmit }) => {
  const [tractorPosition, setTractorPosition] = useState({
    latitude: "",
    longitude: "",
  });

  const [fieldPoints, setFieldPoints] = useState({
    point1: { latitude: "", longitude: "" },
    point2: { latitude: "", longitude: "" },
    point3: { latitude: "", longitude: "" },
    point4: { latitude: "", longitude: "" },
  });

  const handleTractorChange = (e) => {
    setTractorPosition({
      ...tractorPosition,
      [e.target.name]: e.target.value,
    });
  };

  const handleFieldPointChange = (e) => {
    const { name, value } = e.target;
    const [point, coordinate] = name.split("-");

    setFieldPoints((prevState) => ({
      ...prevState,
      [point]: {
        ...prevState[point],
        [coordinate]: value,
      },
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    onSubmit(tractorPosition, fieldPoints);
  };

  return (
    <div className="max-w-2xl mx-auto p-6 bg-white rounded-xl shadow-lg">
      <h1 className="text-2xl font-semibold text-center mb-6">Tractor Form</h1>
      <form onSubmit={handleSubmit}>
        {/* Form Posisi Traktor */}
        <div className="mb-4">
          <label className="block text-lg font-medium text-gray-700">Tractor Position</label>
          <div className="flex gap-4 mt-2">
            <div className="w-full">
              <label className="block text-sm font-medium text-gray-600">Latitude</label>
              <input
                type="text"
                name="latitude"
                value={tractorPosition.latitude}
                onChange={handleTractorChange}
                className="mt-1 block w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500"
              />
            </div>
            <div className="w-full">
              <label className="block text-sm font-medium text-gray-600">Longitude</label>
              <input
                type="text"
                name="longitude"
                value={tractorPosition.longitude}
                onChange={handleTractorChange}
                className="mt-1 block w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500"
              />
            </div>
          </div>
        </div>

        {/* Form Titik Sawah */}
        <div className="mb-6">
          <h2 className="text-lg font-semibold text-gray-800 mb-2">Field Points</h2>
          {[...Array(4)].map((_, index) => {
            const point = `point${index + 1}`;
            return (
              <div className="flex gap-4 mb-4" key={point}>
                <div className="w-full">
                  <label className="block text-sm font-medium text-gray-600">Latitude</label>
                  <input
                    type="text"
                    name={`${point}-latitude`}
                    value={fieldPoints[point].latitude}
                    onChange={handleFieldPointChange}
                    className="mt-1 block w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500"
                  />
                </div>
                <div className="w-full">
                  <label className="block text-sm font-medium text-gray-600">Longitude</label>
                  <input
                    type="text"
                    name={`${point}-longitude`}
                    value={fieldPoints[point].longitude}
                    onChange={handleFieldPointChange}
                    className="mt-1 block w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500"
                  />
                </div>
              </div>
            );
          })}
        </div>

        {/* Submit Button */}
        <div className="flex justify-center">
          <button
            type="submit"
            className="px-6 py-2 bg-blue-600 text-white font-semibold rounded-lg shadow-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-opacity-50"
          >
            Submit
          </button>
        </div>
      </form>
    </div>
  );
};

export default TractorForm;
