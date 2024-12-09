"use client";

import React, { useState, useEffect, useContext } from "react";
import {
  MapContainer,
  TileLayer,
  Marker,
  Popup,
  Polyline,
  useMapEvents,
} from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { useSocket } from "../../contexts/SocketContext";
import { BoatContext } from "../../contexts/BoatContext";
import "./AutonomousControl.css";

function AutonomousControl() {
  const { socket, isConnected, setCommandMode } = useSocket();
  const { boats } = useContext(BoatContext);

  const [targetBoatId, setTargetBoatId] = useState("");
  const [mapCenter, setMapCenter] = useState([37.866942, -122.315452]);
  const [selectedPosition, setSelectedPosition] = useState(null);
  const [boatTrails, setBoatTrails] = useState({});
  const [icons, setIcons] = useState({});

  // Set command mode when component mounts
  useEffect(() => {
    if (setCommandMode) {
      setCommandMode("autonomous");
    }
  }, [setCommandMode]);

  // Dynamically load Leaflet and initialize icons
  useEffect(() => {
    if (typeof window !== "undefined") {
      const L = require("leaflet");

      // Initialize icons
      setIcons({
        boatIcon: new L.Icon({
          iconUrl: "/boat.png", // Ensure this image is in the public directory
          iconSize: [32, 32],
          iconAnchor: [16, 32],
          popupAnchor: [0, -32],
        }),
        selectedBoatIcon: new L.Icon({
          iconUrl: "/boat.png",
          iconSize: [48, 48],
          iconAnchor: [24, 48],
          popupAnchor: [0, -48],
        }),
        selectedIcon: new L.Icon({
          iconUrl: "/target-location.png", // Ensure this image is in the public directory
          iconSize: [32, 32],
          iconAnchor: [16, 32],
          popupAnchor: [0, -32],
        }),
      });
    }
  }, []);

  // Automatically set the first boat as the target
  useEffect(() => {
    if (!targetBoatId && boats.length > 0) {
      setTargetBoatId(boats[0].boat_id);
    }
  }, [boats, targetBoatId]);

  // Update boat trails
  useEffect(() => {
    const newBoatTrails = { ...boatTrails };
    boats.forEach((b) => {
      if (b.lat && b.lng) {
        if (!newBoatTrails[b.boat_id]) {
          newBoatTrails[b.boat_id] = [];
        }
        newBoatTrails[b.boat_id].push([b.lat, b.lng]);
        if (newBoatTrails[b.boat_id].length > 50) {
          newBoatTrails[b.boat_id].shift();
        }
      }
    });
    setBoatTrails(newBoatTrails);
  }, [boats]);

  const handleMapClick = (e) => {
    setSelectedPosition({
      lat: parseFloat(e.latlng.lat.toFixed(6)),
      lng: parseFloat(e.latlng.lng.toFixed(6)),
    });
  };

  const sendRouteToCurrentBoat = () => {
    if (socket && isConnected && targetBoatId && selectedPosition) {
      const data = {
        id: targetBoatId,
        md: "auto",
        tlat: selectedPosition.lat,
        tlng: selectedPosition.lng,
      };
      socket.emit("gui_data", data);
      console.log("Sent route to current boat:", data);
    } else {
      console.warn("Socket is not connected or no boat/position is set.");
    }
  };

  // Custom hook for capturing map events
  function ClickableMap() {
    useMapEvents({
      click: handleMapClick,
    });
    return null;
  }

  return (
    <div className="autonomous-control-container">
      <h2>Autonomous Control</h2>

      {!isConnected && <p className="warning-text">Not connected to server.</p>}

      <div className="boat-selection">
        <label>Select Boat: </label>
        <select
          value={targetBoatId}
          onChange={(e) => setTargetBoatId(e.target.value)}
        >
          {boats.map((b) => (
            <option key={b.boat_id} value={b.boat_id}>
              {b.boat_id}
            </option>
          ))}
        </select>
      </div>

      <MapContainer
        center={mapCenter}
        zoom={14}
        style={{ width: "100%", height: "500px" }}
      >
        <TileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution="&copy; OpenStreetMap contributors"
        />
        <ClickableMap />
        {boats.map((b) => (
          <Marker
            key={b.boat_id}
            position={[b.lat, b.lng]}
            icon={
              b.boat_id === targetBoatId
                ? icons.selectedBoatIcon
                : icons.boatIcon
            }
          >
            <Popup>
              <b>{b.boat_id}</b>
              <br />
              Latitude: {b.lat.toFixed(6)}
              <br />
              Longitude: {b.lng.toFixed(6)}
              <br />
              <button onClick={sendRouteToCurrentBoat}>Send Route</button>
            </Popup>
          </Marker>
        ))}

        {selectedPosition && (
          <Marker position={selectedPosition} icon={icons.selectedIcon}>
            <Popup>
              Selected Position:
              <br />
              Latitude: {selectedPosition.lat.toFixed(6)}
              <br />
              Longitude: {selectedPosition.lng.toFixed(6)}
              <br />
              <button onClick={sendRouteToCurrentBoat}>Send Route</button>
            </Popup>
          </Marker>
        )}

        {Object.keys(boatTrails).map((boatId) => (
          <Polyline
            key={boatId}
            positions={boatTrails[boatId]}
            pathOptions={{ color: "blue" }}
          />
        ))}
      </MapContainer>
    </div>
  );
}

export default AutonomousControl;
