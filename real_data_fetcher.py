
import os
import requests
import time
from typing import List, Dict, Any, Tuple
from joblib import Memory

fetcher_cache = Memory("api_cache", verbose=0)

class RealDataFetcher:
    def __init__(self, api_key: str):
        self.api_key = api_key
        self.base_url = "https://api.tomtom.com/traffic/services/5"

    # Removed cache decorator due to joblib subclass issues
    def fetch_incidents(self, bbox: Tuple[float, float, float, float]) -> List[Dict]:
        """
        bbox: (min_lat, min_lon, max_lat, max_lon)
        """
        if not self.api_key: return []
        
        # TomTom format: minLon,minLat,maxLon,maxLat
        bbox_str = f"{bbox[1]},{bbox[0]},{bbox[3]},{bbox[2]}"
        
        url = f"{self.base_url}/incidentDetails?key={self.api_key}&bbox={bbox_str}&fields={'{incidents{type,geometry{type,coordinates},properties{iconCategory,magnitudeOfDelay,startTime,endTime,events{description,code,iconCategory}}}}'}&language=en-US&t={int(time.time()/60)}" # cache bust per min
        
        try:
            print(f"Fetching Incidents from TomTom: {bbox_str} ...")
            resp = requests.get(url, timeout=5)
            if resp.status_code == 200:
                data = resp.json()
                results = []
                for inc in data.get("incidents", []):
                    props = inc.get("properties", {})
                    geom = inc.get("geometry", {}).get("coordinates", [])
                    if not geom: continue
                    
                    # TomTom uses lon,lat
                    lon, lat = geom[0] if isinstance(geom[0], float) else geom[0] # Handle point or line
                    
                    category = props.get("iconCategory", 0)
                    desc = "Incident"
                    code = 0
                    if "events" in props and props["events"]:
                        event = props["events"][0]
                        desc = event.get("description", desc)
                        code = event.get("code", 0)
                    
                    # Improved Type Mapping
                    # 6=Accident, 8=Roadworks, 9=RoadClosed, 1=Jam
                    mapped_type = "incident"
                    subtype = "Unknown"
                    
                    if category == 6: 
                        mapped_type = "accident"
                        subtype = "Car Crash" # Default, refine if desc contains keywords
                        if "truck" in desc.lower(): subtype = "Truck Collision"
                    elif category == 8: 
                        mapped_type = "works"
                        subtype = "Roadworks"
                    elif category == 9: 
                        mapped_type = "closure"
                        subtype = "Road Closed"
                    elif category == 1:
                        mapped_type = "jam"
                        subtype = "Traffic Jam"
                    
                    results.append({
                        "lat": lat,
                        "lon": lon,
                        "type": mapped_type,
                        "subtype": subtype,
                        "severity": "High" if props.get("magnitudeOfDelay", 0) > 2 else "Medium",
                        "description": desc,
                        "startTime": props.get("startTime", None),
                        "endTime": props.get("endTime", None)
                    })
                return results
            else:
                print(f"TomTom Error {resp.status_code}: {resp.text}")
                return []
        except Exception as e:
            print(f"TomTom Fetch Exception: {e}")
            return []

    def fetch_flow_segment(self, lat: float, lon: float) -> Dict[str, Any]:
        """
        Get speed for specific point
        """
        if not self.api_key: return {}
        url = f"https://api.tomtom.com/traffic/services/4/flowSegmentData/absolute/10/json?key={self.api_key}&point={lat},{lon}"
        try:
             resp = requests.get(url, timeout=3)
             if resp.status_code == 200:
                 return resp.json().get("flowSegmentData", {})
        except:
             pass
        return {}
