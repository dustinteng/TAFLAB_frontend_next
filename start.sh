#!/bin/bash

# Navigate to backend directory and start backendsimple.py
echo "Starting backend..."
cd "$(dirname "$0")/backend"
python3 backendsimple.py &

# Navigate to frontend directory and start npm
echo "Starting frontend..."
cd ../frontend/src
npm start

# Keep track of background process IDs if you want to kill them later
# backend_pid=$!
# echo "Backend is running with PID: $backend_pid"

# Optionally, add a command to kill the backend process later if you wish
# kill $backend_pid
