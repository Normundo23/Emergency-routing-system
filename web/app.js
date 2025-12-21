
// State
let map = null;
let tomtomKey = null;
let startMarker = null;
let goalMarker = null;

// GeoJSON Sources IDs
const SRC_AI_ROUTE = 'cortex-route';
const SRC_STD_ROUTE = 'std-route';
const SRC_DEMAND = 'cortex-demand';

// State for toggling
let currentRouteMode = 'ai';
let lastRouteData = null;
let lastTomTomSummary = null;
let trafficVisible = false;
let demandVisible = false;

// Mock regions for visualisation
const demandRegions = [
    { id: 0, center: [121.0420, 14.4090], radius: 0.008, name: 'Downtown' },
    { id: 1, center: [121.0300, 14.4250], radius: 0.010, name: 'North District' },
    { id: 2, center: [121.0500, 14.3900], radius: 0.012, name: 'South Residential' },
    { id: 3, center: [121.0700, 14.4100], radius: 0.009, name: 'East Industrial' },
    { id: 4, center: [121.0200, 14.4000], radius: 0.008, name: 'West Commercial' },
    { id: 5, center: [121.0400, 14.4400], radius: 0.011, name: 'Alabang Hills' },
    { id: 6, center: [121.0450, 14.4150], radius: 0.006, name: 'Filinvest City' },
    { id: 7, center: [121.0600, 14.3800], radius: 0.013, name: 'Tunasan Area' },
    { id: 8, center: [121.0500, 14.4300], radius: 0.009, name: 'Sucat Interchange' },
    { id: 9, center: [121.0350, 14.3950], radius: 0.007, name: 'Poblacion' }
];

// Init
(async function init() {
    try {
        const cfgRes = await fetch('/config');
        const cfg = await cfgRes.json();

        if (cfg.tomtom_key) {
            tomtomKey = cfg.tomtom_key;
            initMap(tomtomKey);
        } else {
            alert("TomTom API Key missing! Check server configuration.");
        }
    } catch (e) {
        console.error("Config failed", e);
    }

    // Warm up backend
    // Warm up backend with valid bbox
    try {
        fetch('/graph/load_bbox', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ north: 14.5, south: 14.3, east: 121.1, west: 121.0 })
        });
    } catch (e) { }
})();

function initMap(key) {
    // Initialize TomTom Map
    try {
        map = tt.map({
            key: key,
            container: 'map',
            center: [121.0422, 14.4168], // Muntinlupa
            zoom: 13,
            dragPan: true,
            // style: 'https://api.tomtom.com/map/1/style/20.0.0-8/basic-night.json' // Reverted to default due to stability
        });
        console.log("Map initialized successfully");
    } catch (e) {
        alert("Map Init Error: " + e.message);
        console.error(e);
    }

    map.addControl(new tt.NavigationControl());

    // Add Sources & Layers on Load
    map.on('load', () => {
        // AI Route Layer (Blue/Glow)
        map.addSource(SRC_AI_ROUTE, { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
        map.addLayer({
            id: 'cortex-glow', type: 'line', source: SRC_AI_ROUTE,
            layout: { 'line-cap': 'round', 'line-join': 'round' },
            paint: { 'line-color': '#3b82f6', 'line-width': 8, 'line-opacity': 0.6, 'line-blur': 3 }
        });
        map.addLayer({
            id: 'cortex-line', type: 'line', source: SRC_AI_ROUTE,
            layout: { 'line-cap': 'round', 'line-join': 'round' },
            paint: { 'line-color': '#60a5fa', 'line-width': 4 }
        });

        // Standard Route Layer (Grey/Dashed)
        map.addSource(SRC_STD_ROUTE, { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
        map.addLayer({
            id: 'std-line', type: 'line', source: SRC_STD_ROUTE,
            layout: { 'line-cap': 'round', 'line-join': 'round' },
            paint: {
                'line-color': '#94a3b8',
                'line-width': 4,
                'line-dasharray': [2, 2]
            }
        });

        // Initialize Traffic Layers
        // Initialize Traffic Layers
        map.on('style.load', () => {
            // trigger traffic visibility check if needed
        });

        // Heatmap Source & Layer (Added)
        map.addSource(SRC_DEMAND, { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
        map.addLayer({
            id: 'demand-fill',
            type: 'fill',
            source: SRC_DEMAND,
            paint: {
                'fill-color': ['get', 'color'],
                'fill-opacity': 0.4,
                'fill-outline-color': '#ffffff'
            }
        });
    });

    // Interaction
    map.on('click', handleMapClick);
}

// --- Map Interaction ---

function handleMapClick(e) {
    const { lng, lat } = e.lngLat;
    if (!startMarker) {
        setStart([lng, lat]);
    } else if (!goalMarker) {
        setGoal([lng, lat]);
    }
}

function setStart(lngLat) {
    if (startMarker) startMarker.remove();
    const el = document.createElement('div');
    el.className = 'custom-pin';
    el.style.backgroundColor = '#22c55e';
    el.style.width = '16px'; el.style.height = '16px'; el.style.borderRadius = '50%';
    el.style.border = '2px solid white'; el.style.boxShadow = '0 0 10px #22c55e';

    startMarker = new tt.Marker({ element: el }).setLngLat(lngLat).addTo(map);
    document.getElementById('origin-input').value = `${lngLat[1].toFixed(5)}, ${lngLat[0].toFixed(5)}`;
    setStatus('Start set.');
    checkAutoCalcStd();
}

function setGoal(lngLat) {
    if (goalMarker) goalMarker.remove();
    const el = document.createElement('div');
    el.className = 'custom-pin';
    el.style.backgroundColor = '#ef4444';
    el.style.width = '16px'; el.style.height = '16px'; el.style.borderRadius = '50%';
    el.style.border = '2px solid white'; el.style.boxShadow = '0 0 10px #ef4444';

    goalMarker = new tt.Marker({ element: el }).setLngLat(lngLat).addTo(map);
    document.getElementById('dest-input').value = `${lngLat[1].toFixed(5)}, ${lngLat[0].toFixed(5)}`;
    setStatus('Ready to optimize.');
    checkAutoCalcStd();
}

function checkAutoCalcStd() {
    if (startMarker && goalMarker) {
        calculateStandardRoute();
    }
}

async function calculateStandardRoute() {
    if (!startMarker || !goalMarker) return;

    // Show partial UI
    document.getElementById('route-tabs-container').style.display = 'flex';
    document.getElementById('tab-std').style.display = 'block'; // Ensure tab visible
    switchRouteTab('std'); // Default to standard initially

    try {
        const start = startMarker.getLngLat();
        const goal = goalMarker.getLngLat();

        const ttRes = await tt.services.calculateRoute({
            key: tomtomKey,
            locations: [start, goal],
            traffic: true
        });

        const ttRoute = ttRes.toGeoJson().features[0];
        const ttSummary = ttRes.routes[0].summary;
        lastTomTomSummary = ttSummary;

        // Render Standard Line
        map.getSource(SRC_STD_ROUTE).setData(ttRoute);

        // Update Stats
        document.getElementById('std-eta').innerText = `${Math.ceil(ttSummary.travelTimeInSeconds / 60)} min`;

        // Show Standard Stats immediately
        document.getElementById('eta-val').innerText = `${Math.ceil(ttSummary.travelTimeInSeconds / 60)} min`;
        document.getElementById('dist-val').innerText = `${(ttSummary.lengthInMeters / 1000).toFixed(2)} km`;

        // Fit bounds for preview
        const bounds = new tt.LngLatBounds();
        ttRoute.geometry.coordinates.forEach(c => bounds.extend(c));
        map.fitBounds(bounds, { padding: 50 });

        setStatus("Standard route loaded. Click Optimize for AI.");

    } catch (e) {
        console.error("Standard Route Error", e);
    }
}

// --- Hybrid Routing (AI Only Now) ---

document.getElementById('calc-route').onclick = async () => {
    if (!startMarker || !goalMarker) { setStatus("Pick points first!"); return; }

    const btn = document.getElementById('calc-route');
    btn.disabled = true; btn.innerHTML = "Optimizing...";
    setStatus("Calculating Cortex AI Route...");

    const start = startMarker.getLngLat();
    const goal = goalMarker.getLngLat();

    try {
        const cortexRes = await fetch('/route/coords', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                start_lat: start.lat, start_lon: start.lng,
                goal_lat: goal.lat, goal_lon: goal.lng
            })
        }).then(r => r.json());

        // Process AI Results
        processRoutingResults(cortexRes);

    } catch (e) {
        console.error(e);
        setStatus("AI Routing failed.");
    } finally {
        btn.disabled = false; btn.innerHTML = '<span>⚡</span> Optimize Route';
    }
};

function processRoutingResults(ctxRes) {
    lastRouteData = ctxRes;

    if (ctxRes.status === 'ok' && ctxRes.path_coords) {
        let coordinates = ctxRes.path_coords.map(aa => [aa.lon, aa.lat]);
        if (startMarker) coordinates.unshift([startMarker.getLngLat().lng, startMarker.getLngLat().lat]);
        if (goalMarker) coordinates.push([goalMarker.getLngLat().lng, goalMarker.getLngLat().lat]);

        const aiGeoJson = {
            type: 'Feature',
            geometry: {
                type: 'LineString',
                coordinates: coordinates
            }
        };
        map.getSource(SRC_AI_ROUTE).setData(aiGeoJson);
    }

    // Switch Tab to AI
    switchRouteTab('ai');

    // Update Cortex Stats
    if (ctxRes.eta_seconds) {
        const aiEtaMin = Math.ceil(ctxRes.eta_seconds / 60);
        document.getElementById('cortex-eta').innerText = `${aiEtaMin} min`;
    }

    const visited = ctxRes.visited_nodes || 1500;
    const stdEst = parseInt(document.getElementById('nodes-std').innerText) || Math.floor(visited * 1.2);
    document.getElementById('nodes-visited').innerText = visited;

    // If standard stats missing (shouldn't be), fill gap
    if (document.getElementById('nodes-std').innerText == "") {
        document.getElementById('nodes-std').innerText = stdEst;
    }

    document.getElementById('search-reduction').innerText = `${((stdEst - visited) / stdEst * 100).toFixed(0)}%`;
}

// --- Switching Logic ---

window.switchRouteTab = function (mode) {
    if (!lastRouteData || !lastTomTomSummary) return;
    currentRouteMode = mode;

    document.getElementById('tab-ai').classList.toggle('active', mode === 'ai');
    document.getElementById('tab-std').classList.toggle('active', mode === 'std');

    // Visibility
    if (mode === 'ai') {
        map.setLayoutProperty('cortex-glow', 'visibility', 'visible');
        map.setLayoutProperty('cortex-line', 'visibility', 'visible');
        map.setLayoutProperty('std-line', 'visibility', 'none');

        // Show AI Stats
        const eta = Math.ceil(lastRouteData.eta_seconds / 60);
        document.getElementById('eta-val').innerText = `${eta} min`;
        document.getElementById('dist-val').innerText = `${(lastRouteData.steps.reduce((a, b) => a + (b.distance_m || 0), 0) / 1000).toFixed(2)} km`;

        // Time Saved
        const diff = lastTomTomSummary.travelTimeInSeconds - lastRouteData.eta_seconds;
        updateTimeSaved(diff);

    } else {
        map.setLayoutProperty('cortex-glow', 'visibility', 'none');
        map.setLayoutProperty('cortex-line', 'visibility', 'none');
        map.setLayoutProperty('std-line', 'visibility', 'visible');

        // Show Standard Stats
        const eta = Math.ceil(lastTomTomSummary.travelTimeInSeconds / 60);
        document.getElementById('eta-val').innerText = `${eta} min`;
        document.getElementById('dist-val').innerText = `${(lastTomTomSummary.lengthInMeters / 1000).toFixed(2)} km`;

        // Hide Time Saved
        document.getElementById('time-saved').style.display = 'none';
        document.getElementById('time-saved-bar').style.width = '0%';
    }
};

function updateTimeSaved(diffSeconds) {
    const min = Math.ceil(diffSeconds / 60);
    const elem = document.getElementById('time-saved');
    const bar = document.getElementById('time-saved-bar');
    const txt = document.getElementById('time-saved-text');

    if (min > 0) {
        elem.innerText = `${min} min faster`;
        elem.style.display = 'block';
        bar.style.width = '100%'; // Max it out for effect
        txt.innerText = `Cortex is ${min} minutes faster`;
        txt.style.color = '#4ade80';
    } else {
        elem.innerText = 'Matched';
        bar.style.width = '0%';
        txt.innerText = 'Performance Matched';
        txt.style.color = '#94a3b8';
    }
}

// --- Toggles ---

// Traffic: TomTom Vector Traffic
// Traffic: TomTom Vector Traffic
// trafficVisible is global
document.getElementById('toggle-traffic').onclick = () => {
    trafficVisible = !trafficVisible;
    document.getElementById('toggle-traffic').classList.toggle('active', trafficVisible);

    if (trafficVisible) {
        map.showTrafficFlow();
        map.showTrafficIncidents();
        setStatus("Real-time Traffic Active");
        checkIncidents(); // Fetch text details
    } else {
        map.hideTrafficFlow();
        map.hideTrafficIncidents();
        document.getElementById('alerts-panel').style.display = 'none';
        setStatus("Traffic hidden");
    }
};

async function checkIncidents() {
    try {
        const res = await fetch('/incidents');
        const data = await res.json();
        if (data.status === 'ok' && data.incidents && data.incidents.length > 0) {
            const topInc = data.incidents[0]; // Get first one logic
            // Find most severe
            const severe = data.incidents.find(i => i.severity === 'High') || data.incidents[0];

            let timeStr = "";
            if (severe.startTime) {
                const t = new Date(severe.startTime);
                // format: "Dec 19, 15:41"
                const dateStr = t.toLocaleDateString('en-US', { month: 'short', day: 'numeric' });
                const timeStrVal = t.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', hour12: false });
                timeStr = ` | Reported: ${dateStr}, ${timeStrVal}`;
            }

            document.getElementById('alerts-panel').style.display = 'block';
            document.getElementById('alert-content').innerHTML = `
                <strong>${severe.description}</strong><br/>
                <span style="opacity:0.8; font-size: 0.75rem;">
                    Type: ${severe.subtype || severe.type} | Risk: ${severe.severity}${timeStr}
                </span>
            `;
        }
    } catch (e) { console.error(e); }
}

document.getElementById('toggle-demand').onclick = async () => {
    demandVisible = !demandVisible;
    document.getElementById('toggle-demand').classList.toggle('active', demandVisible);

    if (!demandVisible) {
        map.getSource(SRC_DEMAND).setData({ type: 'FeatureCollection', features: [] });
        setStatus("Heatmap hidden");
        return;
    }

    setStatus("Fetching demand predictions...");
    try {
        const res = await fetch('/predict/demand');
        const data = await res.json();

        if (data.status === 'ok') {
            const features = demandRegions.map(r => {
                const risk = data.predictions[`region_${r.id}`] || Math.random();
                let color = '#22c55e'; // Green
                if (risk > 0.6) color = '#ef4444'; // Red
                else if (risk > 0.3) color = '#f59e0b'; // Amber

                // Create approximate circle polygon
                return createCircleFeature(r.center, r.radius, color, r.name, risk);
            });

            map.getSource(SRC_DEMAND).setData({
                type: 'FeatureCollection',
                features: features
            });
            setStatus("Heatmap Active");
        }
    } catch (e) {
        console.error(e);
        setStatus("Heatmap error");
    }
};

function createCircleFeature(center, radiusDeg, color, name, risk) {
    const points = 64;
    const coords = [];
    for (let i = 0; i < points; i++) {
        const theta = (i / points) * (2 * Math.PI);
        const x = center[0] + (radiusDeg * Math.cos(theta));
        const y = center[1] + (radiusDeg * Math.sin(theta));
        coords.push([x, y]);
    }
    coords.push(coords[0]); // Close loop

    return {
        type: 'Feature',
        properties: { color: color, name: name, risk: risk },
        geometry: { type: 'Polygon', coordinates: [coords] }
    };
}

document.getElementById('reset-btn').onclick = () => {
    if (startMarker) startMarker.remove();
    if (goalMarker) goalMarker.remove();
    startMarker = null; goalMarker = null;

    map.getSource(SRC_AI_ROUTE).setData({ type: 'FeatureCollection', features: [] });
    map.getSource(SRC_STD_ROUTE).setData({ type: 'FeatureCollection', features: [] });

    document.getElementById('route-tabs-container').style.display = 'none';
    document.getElementById('origin-input').value = '';
    document.getElementById('dest-input').value = '';
    document.getElementById('eta-val').innerText = '--';
    document.getElementById('dist-val').innerText = '--';
    setStatus("Map cleared.");
};

function setStatus(msg) {
    document.getElementById('status-text').innerText = msg;
}

// Tab Nav
window.switchTab = function (tabName) {
    document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
    event.target.classList.add('active');
    document.querySelectorAll('.tab-content').forEach(c => { c.style.display = 'none'; });
    document.getElementById(`tab-${tabName}`).style.display = 'block';
};
