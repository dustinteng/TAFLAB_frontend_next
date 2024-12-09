import "../globals.css";
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
                <div>
                  <header>Your Header Here</header>
                  <main>{children}</main>
                </div>
              </RecordingProvider>
            </BoatProvider>
          </SocketProvider>
        </ThemeProvider>
      </body>
    </html>
  );
}
