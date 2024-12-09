"use client";

import "../globals.css";
import Header from "../components/Header/Header"; // Import your Header component
import { BoatProvider } from "../contexts/BoatContext";
import { SocketProvider } from "../contexts/SocketContext";
import { RecordingProvider } from "../contexts/RecordingContext";
import { ThemeProvider } from "../contexts/ThemeContext";

export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <body>
        <ThemeProvider>
          <SocketProvider>
            <BoatProvider>
              <RecordingProvider>
                {/* Use the Header component */}
                <Header />
                <main>{children}</main>
              </RecordingProvider>
            </BoatProvider>
          </SocketProvider>
        </ThemeProvider>
      </body>
    </html>
  );
}
