document.addEventListener('DOMContentLoaded', function() {
    let connectionStatus = false;
        
    function getRobotIP() {
        return document.getElementById('robotIP').value;
    }

    // Initialize map centered on some default coords
    const map = L.map('map').setView([9.0563, 7.4985], 13);

    // Add OpenStreetMap tiles
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    let marker = L.marker([9.0563, 7.4985]).addTo(map);
        
    function fetchGPS() {
        fetch("http://192.168.4.1/gps")   // ESP AP IP
            .then(res => res.json())
            .then(data => {
            document.getElementById("coords").innerText =
                `Lat: ${data.lat}, Lng: ${data.lng}`;
            
            // Update marker position
            marker.setLatLng([data.lat, data.lng]);

            // Center map on new position
            map.setView([data.lat, data.lng]);
            })
            .catch(err => console.error("Error:", err));
    }

    // Poll every 2 seconds
    setInterval(fetchGPS, 5000);

    // Fetch immediately on page load
    fetchGPS();

    function getSpeed() {
        return parseInt(document.getElementById('speedSlider').value, 10);
    }
    function getRadius() {
    return parseFloat(document.getElementById('radiusInput').value);
    }
    function getDepth() {
        return parseFloat(document.getElementById('depthInput').value);
    }

    // Planting mode state
    let plantingMode = false;

    // Toggle planting mode
    function togglePlantMode() {
        const btn = document.getElementById('plantToggle');
        plantingMode = !plantingMode;
        if (plantingMode) {
            btn.innerHTML = "⏸️<br>PAUSE PLANT";
            btn.classList.remove('paused');
            btn.classList.add('planting');
            sendCommand('plant');
        } else {
            btn.innerHTML = "🌱<br>PLANT";
            btn.classList.remove('planting');
            btn.classList.add('paused');
            sendCommand('pause');
        }
    }

    async function sendCommand(command) {
        const ip = getRobotIP();
        const speed = getSpeed();
        const radius = getRadius();
        const depth = getDepth();
        const timestamp = new Date().toLocaleTimeString();

        log(`[${timestamp}] Sending: ${command.toUpperCase()} @ ${speed}% | radius=${radius} | depth=${depth}`);

        // Build query string
        const params = `speed=${speed}&radius=${radius}&depth=${depth}`;
        const url = `http://${ip}/${command}?${params}`;
        log(`[${timestamp}] URL: ${url}`);

        try {
            const response = await fetch(url, {
                method: 'GET',
                mode: 'cors'
            });

            if (response.ok) {
                let resultText = await response.text();
                log(`[${timestamp}] ✅ SUCCESS: ${resultText}`);
                updateConnectionStatus(true);
            } else {
                log(`[${timestamp}] ❌ ERROR: HTTP ${response.status}`);
                updateConnectionStatus(false);
            }

        } catch (error) {
            log(`[${timestamp}] ❌ FAILED: ${error.message}`);
            updateConnectionStatus(false);
        }
    }
    
    function updateConnectionStatus(connected) {
        connectionStatus = connected;
        const indicator = document.getElementById('connectionStatus');
        const message = document.getElementById('statusMessage');
        
        if (connected) {
            indicator.classList.add('connected');
            message.textContent = 'Connected to robot ✅';
            message.style.color = '#4CAF50';
        } else {
            indicator.classList.remove('connected');
            message.textContent = 'Connection failed ❌';
            message.style.color = '#f44336';
        }
    }
    
    function log(message) {
        const logElement = document.getElementById('commandLog');
        logElement.innerHTML += message + '<br>';
        logElement.scrollTop = logElement.scrollHeight;
    }
    
    // Test connection on page load
    setTimeout(() => {
        log('Testing connection...');
    }, 1000);
    
    // Keyboard controls (optional)
    document.addEventListener('keydown', function(event) {
        switch(event.code) {
            case 'ArrowUp':
                sendCommand('forward');
                break;
            case 'ArrowDown':
                sendCommand('backward');
                break;
            case 'ArrowLeft':
                sendCommand('left');
                break;
            case 'ArrowRight':
                sendCommand('right');
                break;
            case 'Space':
                sendCommand('stop');
                event.preventDefault();
                break;
        }
    });

    const speedSlider = document.getElementById('speedSlider');
    const speedValue = document.getElementById('speedValue');
    speedSlider.addEventListener('input', function() {
        speedValue.textContent = speedSlider.value + '%';
    });
});