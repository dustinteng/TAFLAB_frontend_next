"use client";

import React, { useEffect, useState, useRef, useContext } from "react";
import dynamic from "next/dynamic";
import "leaflet/dist/leaflet.css";
import { BoatContext } from "../../contexts/BoatContext";
import "./HeatmapWindow.css";

// Dynamically import React-Leaflet components
const MapContainer = dynamic(
  () => import("react-leaflet").then((mod) => mod.MapContainer),
  { ssr: false }
);
const TileLayer = dynamic(
  () => import("react-leaflet").then((mod) => mod.TileLayer),
  { ssr: false }
);
const Marker = dynamic(
  () => import("react-leaflet").then((mod) => mod.Marker),
  { ssr: false }
);
const Popup = dynamic(() => import("react-leaflet").then((mod) => mod.Popup), {
  ssr: false,
});
const useMap = dynamic(
  () => import("react-leaflet").then((mod) => mod.useMap),
  { ssr: false }
);

// Import Leaflet for useEffect
const HeatmapWindow = () => {
  const { boats } = useContext(BoatContext);
  const [heatmapData, setHeatmapData] = useState([]);
  const [dataType, setDataType] = useState("temperature");
  const legendRef = useRef(null);
  const [legendPosition, setLegendPosition] = useState({ x: null, y: null });

  const isDragging = useRef(false);
  const dragStartPos = useRef({ x: 0, y: 0 });
  const legendStartPos = useRef({ x: 0, y: 0 });

  const [boatIcon, setBoatIcon] = useState(null);

  // Dynamically initialize Leaflet and custom icons
  useEffect(() => {
    if (typeof window !== "undefined") {
      const L = require("leaflet");
      require("leaflet.heat");

      setBoatIcon(
        new L.Icon({
          iconUrl: "/boat.png",
          iconSize: [32, 32],
          iconAnchor: [16, 32],
          popupAnchor: [0, -32],
        })
      );
    }
  }, []);

  useEffect(() => {
    if (boats.length > 0) {
      const data = boats.map((boat) => {
        const lat = boat.lat !== undefined ? boat.lat : 0;
        const lng = boat.lng !== undefined ? boat.lng : 0;
        let value = dataType === "chaos" ? boat.chaos : boat.temperature;
        return [lat, lng, value];
      });
      setHeatmapData(data);
    }
  }, [boats, dataType]);

  const HeatmapLayer = () => {
    const map = useMap();

    useEffect(() => {
      if (heatmapData.length > 0 && typeof window !== "undefined") {
        const L = require("leaflet");
        const heatLayer = L.heatLayer(heatmapData, {
          radius: 50,
          blur: 50,
          maxZoom: 10,
        }).addTo(map);

        return () => {
          map.removeLayer(heatLayer);
        };
      }
    }, [map, heatmapData]);

    return null;
  };

  const BoatMarkers = () => {
    return (
      <>
        {boats.map((boat) => {
          const lat = boat.lat !== undefined ? boat.lat : 0;
          const lng = boat.lng !== undefined ? boat.lng : 0;

          return (
            <Marker key={boat.boat_id} position={[lat, lng]} icon={boatIcon}>
              <Popup>
                <b>{boat.boat_id}</b>
                <br />
                Latitude: {lat.toFixed(6)}
                <br />
                Longitude: {lng.toFixed(6)}
              </Popup>
            </Marker>
          );
        })}
      </>
    );
  };

  const ColorScaleLegend = () => {
    useEffect(() => {
      if (
        legendRef.current &&
        legendPosition.x === null &&
        legendPosition.y === null
      ) {
        const mapContainer = document.querySelector(".leaflet-container");
        const mapRect = mapContainer.getBoundingClientRect();
        const legendRect = legendRef.current.getBoundingClientRect();

        const x = mapRect.right - legendRect.width - 20;
        const y = mapRect.bottom - legendRect.height - 20;

        setLegendPosition({ x, y });
      }
    }, [legendPosition.x, legendPosition.y]);

    const handleMouseDown = (e) => {
      isDragging.current = true;
      dragStartPos.current = { x: e.clientX, y: e.clientY };
      legendStartPos.current = { x: legendPosition.x, y: legendPosition.y };

      document.addEventListener("mousemove", handleMouseMove);
      document.addEventListener("mouseup", handleMouseUp);

      e.preventDefault();
    };

    const handleMouseMove = (e) => {
      if (isDragging.current) {
        const dx = e.clientX - dragStartPos.current.x;
        const dy = e.clientY - dragStartPos.current.y;

        let newX = legendStartPos.current.x + dx;
        let newY = legendStartPos.current.y + dy;

        const mapContainer = document.querySelector(".leaflet-container");
        const mapRect = mapContainer.getBoundingClientRect();
        const legendRect = legendRef.current.getBoundingClientRect();

        const minX = mapRect.left;
        const maxX = mapRect.right - legendRect.width;
        const minY = mapRect.top;
        const maxY = mapRect.bottom - legendRect.height;

        newX = Math.max(minX, Math.min(newX, maxX));
        newY = Math.max(minY, Math.min(newY, maxY));

        setLegendPosition({ x: newX, y: newY });
      }
    };

    const handleMouseUp = () => {
      isDragging.current = false;
      document.removeEventListener("mousemove", handleMouseMove);
      document.removeEventListener("mouseup", handleMouseUp);
    };

    return (
      <div
        ref={legendRef}
        className="color-scale-legend"
        onMouseDown={handleMouseDown}
        style={{
          top: `${legendPosition.y}px`,
          left: `${legendPosition.x}px`,
        }}
      >
        <strong>
          {dataType === "chaos" ? "Chaos Intensity" : "Temperature"}
        </strong>
        <div
          className={`legend-scale ${
            dataType === "chaos" ? "chaos-scale" : "temperature-scale"
          }`}
        ></div>
        <div className="legend-labels">
          <span>Low</span>
          <span>High</span>
        </div>
      </div>
    );
  };

  const handleDataTypeChange = (event) => {
    setDataType(event.target.value);
  };

  return (
    <div className="heatmap-container">
      <div className="data-type-selector">
        <label>Select Data Type: </label>
        <select value={dataType} onChange={handleDataTypeChange}>
          <option value="chaos">Chaos Intensity</option>
          <option value="temperature">Temperature</option>
        </select>
      </div>
      <MapContainer
        center={[37.866942, -122.315452]}
        zoom={14}
        style={{ height: "600px", width: "100%" }}
        zoomControl={false}
        scrollWheelZoom={false}
        doubleClickZoom={false}
        touchZoom={false}
        dragging={true}
      >
        <TileLayer
          attribution="&copy; OpenStreetMap contributors"
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <HeatmapLayer />
        <BoatMarkers />
      </MapContainer>
      <ColorScaleLegend />
    </div>
  );
};

export default HeatmapWindow;
