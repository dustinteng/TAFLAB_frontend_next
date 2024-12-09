import "../styles/styles.css";
import { BoatProvider } from "../contexts/BoatContext";
import { SocketProvider } from "../contexts/SocketContext";
import { RecordingProvider } from "../contexts/RecordingContext";
import { ThemeProvider } from "../contexts/ThemeContext";

export default function MyApp({ Component, pageProps }) {
  return (
    <ThemeProvider>
      <SocketProvider>
        <BoatProvider>
          <RecordingProvider>
            <Component {...pageProps} />
          </RecordingProvider>
        </BoatProvider>
      </SocketProvider>
    </ThemeProvider>
  );
}
