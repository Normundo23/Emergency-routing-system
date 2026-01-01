
// State
let map = null;
let tomtomKey = null;
let startMarker = null;
let goalMarker = null;

// API Configuration
// When on GitHub Pages, we need to point to the local Python server
// If served from localhost:8000, we can use relative paths, but absolute is safer for hybrid setup
const API_BASE = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
    ? ''
    : 'https://emergency-routing-system.onrender.com';

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

const DEFAULT_KEY = "YJ7ckNeXM9KeiCuC7DYuqo6JWvBOzgDl"; // Fallback for demo/mobile

// Init
(async function init() {
    try {
        const cfgRes = await fetch(`${API_BASE}/config`);
        if (!cfgRes.ok) throw new Error("Backend unreachable");

        const cfg = await cfgRes.json();

        if (cfg.tomtom_key) {
            tomtomKey = cfg.tomtom_key;
            initMap(tomtomKey);
        } else {
            console.warn("No key from backend, using default.");
            initMap(DEFAULT_KEY);
        }
    } catch (e) {
        console.error("Config failed or backend offline. Using default key.", e);
        // Fallback init so map shows up
        tomtomKey = DEFAULT_KEY;
        initMap(tomtomKey);
        setStatus("Backend Offline: Routing Disabled");
        document.getElementById('alerts-panel').style.display = 'block';
        document.getElementById('alert-content').innerText = "Cannot connect to server. Map is in offline mode.";
    }

    // Warm up backend
    try {
        fetch(`${API_BASE}/graph/load_bbox`, {
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

    // Setup Autocomplete
    setupAutocomplete('origin-input', 'origin-suggestions', setStart);
    setupAutocomplete('dest-input', 'dest-suggestions', setGoal);

    // Setup My Location
    document.getElementById('use-my-location').onclick = () => {
        const btn = document.getElementById('use-my-location');
        if (!navigator.geolocation) {
            setStatus("Geolocation not supported by browser.");
            return;
        }

        btn.disabled = true;
        setStatus("Locating you...");

        navigator.geolocation.getCurrentPosition(
            (pos) => {
                const lng = pos.coords.longitude;
                const lat = pos.coords.latitude;
                console.log("Got Location:", lat, lng);

                // Set start point
                const coords = [lng, lat];
                setStart(coords);
                document.getElementById('origin-input').value = "My Location";

                // Fly to user
                map.flyTo({ center: coords, zoom: 14 });
                setStatus("Location found.");
                btn.disabled = false;
            },
            (err) => {
                console.error(err);
                setStatus("Location access denied or error.");
                btn.disabled = false;
            },
            { enableHighAccuracy: true, timeout: 5000, maximumAge: 0 }
        );
    };
}

// --- Autocomplete Logic ---
function debounce(func, wait) {
    let timeout;
    return function (...args) {
        clearTimeout(timeout);
        timeout = setTimeout(() => func.apply(this, args), wait);
    };
}

function setupAutocomplete(inputId, listId, setFunction) {
    const input = document.getElementById(inputId);
    const list = document.getElementById(listId);

    const performSearch = async (val) => {
        if (val.length < 3) {
            list.style.display = 'none';
            return;
        }

        try {
            const response = await tt.services.fuzzySearch({
                key: tomtomKey,
                query: val,
                center: map ? map.getCenter() : undefined,
                countrySet: 'PH',
                limit: 5
            });

            list.innerHTML = '';

            if (response.results && response.results.length > 0) {
                list.style.display = 'block';
                response.results.forEach(result => {
                    const item = document.createElement('div');
                    item.className = 'suggestion-item';

                    const name = result.poi ? result.poi.name : result.address.freeformAddress;
                    const address = result.address.freeformAddress;
                    // If name is same as address, just show one
                    const display = (result.poi && name !== address)
                        ? `<div style="display:flex; flex-direction:column;"><strong>${name}</strong><span style="font-size:0.75em; opacity:0.7">${address}</span></div>`
                        : `<span>${address}</span>`;

                    item.innerHTML = `<span class="suggestion-icon">📍</span> ${display}`;

                    item.onclick = (e) => {
                        e.stopPropagation();
                        const text = result.poi ? name : address;
                        const coords = [result.position.lng, result.position.lat];

                        // setFunction (setStart/setGoal) normally updates input to coordinates
                        // We want to keep the friendly name
                        setFunction(coords);
                        input.value = text;

                        list.style.display = 'none';
                    };
                    list.appendChild(item);
                });
            } else {
                list.style.display = 'none';
            }
        } catch (e) {
            console.error("Autocomplete error", e);
        }
    };

    const debouncedSearch = debounce((e) => performSearch(e.target.value), 300);

    input.addEventListener('input', debouncedSearch);
    input.addEventListener('focus', (e) => { if (e.target.value.length >= 3) performSearch(e.target.value); });

    // Hide on click outside
    document.addEventListener('click', (e) => {
        if (e.target !== input && e.target !== list) {
            list.style.display = 'none';
        }
    });
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
    const btn = document.getElementById('calc-route');
    const originalText = btn.innerHTML;

    // Helper to get input values
    const originText = document.getElementById('origin-input').value.trim();
    const destText = document.getElementById('dest-input').value.trim();

    if (!originText || !destText) {
        setStatus("Please enter or pick both locations.");
        return;
    }

    try {
        btn.disabled = true;
        btn.innerHTML = "Searching...";
        setStatus(" resolving locations...");

        // 1. Resolve Origin if needed
        // If we have a marker AND the text matches the marker coords (roughly), skip.
        // But easier: if text doesn't look like coords or we just want to be safe, resolve it if it's not empty.
        // Actually, if user picked point, text is "lat, lon".
        // Use regex to detect "lat, lon" format to avoid re-geocoding coordinates.
        const coordRegex = /^-?\d+(\.\d+)?,\s*-?\d+(\.\d+)?$/;

        if (!coordRegex.test(originText)) {
            setStatus("Finding origin...");
            const startLoc = await resolveLocation(originText);
            if (!startLoc) throw new Error(`Location not found: ${originText}`);
            console.log("Resolved Start:", startLoc);
            setStart(startLoc); // This updates marker and text field (careful not to loop?)
            // setStart updates text field to coords. That's fine.
        }

        // 2. Resolve Destination if needed
        if (!coordRegex.test(destText)) {
            setStatus("Finding destination...");
            const goalLoc = await resolveLocation(destText);
            if (!goalLoc) throw new Error(`Location not found: ${destText}`);
            setGoal(goalLoc);
        }

        // Slight delay to allow UI to update (markers to drop)
        await new Promise(r => setTimeout(r, 100));

        if (!startMarker || !goalMarker) {
            throw new Error("Could not set points.");
        }

        // 3. Proceed with Routing
        btn.innerHTML = "Optimizing...";
        setStatus("Calculating Cortex AI Route...");

        const start = startMarker.getLngLat();
        const goal = goalMarker.getLngLat();

        const cortexRes = await fetch(`${API_BASE}/route/coords`, {
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
        setStatus("Error: " + e.message);
    } finally {
        btn.disabled = false; btn.innerHTML = originalText;
    }
};

async function resolveLocation(query) {
    // Basic caching could go here
    try {
        const response = await tt.services.fuzzySearch({
            key: tomtomKey,
            query: query,
            center: map.getCenter(),
            countrySet: 'PH' // Bias to Philippines
        });

        if (response.results && response.results.length > 0) {
            const best = response.results[0];
            return [best.position.lng, best.position.lat];
        }
    } catch (e) {
        console.error("Geocoding failed", e);
    }
    return null;
}

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
        document.getElementById('eta-val').innerText = `${eta} min`;

        const distKm = (lastRouteData.steps && Array.isArray(lastRouteData.steps))
            ? (lastRouteData.steps.reduce((a, b) => a + (b.distance_m || 0), 0) / 1000).toFixed(2)
            : '0.00';

        document.getElementById('dist-val').innerText = `${distKm} km`;

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
        const res = await fetch(`${API_BASE}/incidents`);
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
        const res = await fetch(`${API_BASE}/predict/demand`);
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
