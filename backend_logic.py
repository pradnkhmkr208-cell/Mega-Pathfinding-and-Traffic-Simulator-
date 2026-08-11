import heapq
from collections import deque
import requests
import random
import json

# =======================================================
# FEATURE 10: MOCK CENTRAL CLOUD SERVER (Production Database)
# =======================================================
cloud_server_database = {
    "global_traffic_hazards": {
        "Petrol_Pump": "Heavy Accident Delay - Avoid Grid Access",
        "Hospital": "Construction Zone Closed"
    },
    "last_updated_by_vehicle": "SYSTEM_INIT"
}

# =======================================================
# PART 1: CITY GRAPH NETWORK MATRIX
# =======================================================
city_map = {
    'Driver_Spot': [('Hotel', 3), ('Petrol_Pump', 4), ('Airport', 12)],
    'Hotel': [('Driver_Spot', 3), ('Hospital', 5), ('Shopping_Mall', 2), ('Charging_Station', 4)],
    'Petrol_Pump': [('Driver_Spot', 4), ('Garage', 2), ('Bus_Station', 6)],
    'Hospital': [('Hotel', 5), ('Destination', 8), ('Pharmacy', 1)],
    'Garage': [('Petrol_Pump', 2), ('Destination', 7), ('Railway_Station', 9)],
    
    'Charging_Station': [('Hotel', 4), ('Highway_Exit_1', 5)],
    
    'Airport': [('Driver_Spot', 12), ('Tech_Park', 6), ('Highway_Exit_1', 8)],
    'Tech_Park': [('Airport', 6), ('Shopping_Mall', 4), ('Residential_Hub', 5)],
    'Shopping_Mall': [('Hotel', 2), ('Tech_Park', 4), ('Movie_Theater', 2)],
    'Bus_Station': [('Petrol_Pump', 6), ('Railway_Station', 3), ('Market_Yard', 4)],
    'Railway_Station': [('Garage', 9), ('Bus_Station', 3), ('Police_Station', 5)],
    
    'Pharmacy': [('Hospital', 1), ('Police_Station', 4), ('City_Library', 3)],
    'Police_Station': [('Railway_Station', 5), ('Pharmacy', 4), ('Destination', 4), ('Fire_Station', 2)],
    'Movie_Theater': [('Shopping_Mall', 2), ('Central_Park', 3)],
    'Central_Park': [('Movie_Theater', 3), ('Residential_Hub', 4), ('University_Campus', 5)],
    'Residential_Hub': [('Tech_Park', 5), ('Central_Park', 4), ('Highway_Exit_2', 6)],
    'University_Campus': [('Central_Park', 5)],
    
    'Market_Yard': [('Bus_Station', 4), ('Industrial_Zone', 7)],
    'Industrial_Zone': [('Market_Yard', 7), ('Highway_Exit_1', 5)],
    'Highway_Exit_1': [('Airport', 8), ('Industrial_Zone', 5), ('Highway_Exit_2', 15), ('Charging_Station', 5)],
    'Highway_Exit_2': [('Residential_Hub', 6), ('Highway_Exit_1', 15), ('Fire_Station', 8)],
    'Fire_Station': [('Police_Station', 2), ('Highway_Exit_2', 8), ('City_Library', 4)],
    'City_Library': [('Pharmacy', 3), ('Fire_Station', 4)],
    
    'Destination': [('Hospital', 8), ('Garage', 7), ('Police_Station', 4)]
}

def heuristic_estimate(current_node, target_node):
    simulated_coordinates = {
        'Driver_Spot': 15, 'Hotel': 12, 'Petrol_Pump': 11, 'Airport': 10,
        'Shopping_Mall': 8, 'Tech_Park': 7, 'Bus_Station': 9, 'Hospital': 5,
        'Garage': 6, 'Pharmacy': 4, 'Railway_Station': 6, 'Police_Station': 3,
        'Movie_Theater': 6, 'Central_Park': 5, 'Residential_Hub': 4, 'University_Campus': 5,
        'Market_Yard': 8, 'Industrial_Zone': 7, 'Highway_Exit_1': 8, 'Highway_Exit_2': 6,
        'Fire_Station': 4, 'City_Library': 3, 'Charging_Station': 6, 'Destination': 0
    }
    return simulated_coordinates.get(current_node, 10)


# =======================================================
# CORE ARCHITECTURE: DEFINING ALL REQUIRED FUNCTIONS
# =======================================================

# FEATURE 1: Original Dijkstra's Algorithm (Concentric Network Router)
def find_shortest_path_dijkstra(graph, start, end, local_active_hazards):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start, [start])]
    
    while priority_queue:
        current_distance, current_node, path = heapq.heappop(priority_queue)
        if current_node == end:
            return path, distances[current_node]
            
        if current_distance > distances[current_node]:
            continue
            
        for neighbor, weight in graph.get(current_node, []):
            traffic_delay = random.randint(1, 5)
            if neighbor in local_active_hazards:
                traffic_delay += 25  
            distance = current_distance + weight + traffic_delay
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor, path + [neighbor]))
    return None, float('inf')


# FEATURE 2: Advanced A* Search Algorithm (Fast Point-to-Point Pathfinding)
def find_shortest_path_astar(graph, start, end, local_active_hazards):
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    start_f_score = 0 + heuristic_estimate(start, end)
    priority_queue = [(start_f_score, start, [start])]
    
    while priority_queue:
        current_f, current_node, path = heapq.heappop(priority_queue)
        if current_node == end:
            return path, g_score[end]
            
        for neighbor, weight in graph.get(current_node, []):
            traffic_delay = random.randint(1, 5)
            
            # FEATURE 9: DYNAMIC WEIGHTS - Injecting server weight penalties dynamically
            if neighbor in local_active_hazards:
                traffic_delay += 25  
                    
            tentative_g_score = g_score[current_node] + weight + traffic_delay
            if tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic_estimate(neighbor, end)
                heapq.heappush(priority_queue, (f_score, neighbor, path + [neighbor]))
    return None, float('inf')


# FEATURE 3: Greedy Traveling Salesman Problem (Multi-Point Delivery Sequence via Dijkstra Core)
def optimize_delivery_route_tsp(graph, start_node, delivery_locations, local_active_hazards):
    unvisited = set(delivery_locations)
    current = start_node
    optimized_path = [start_node]
    total_tsp_distance = 0
    
    while unvisited:
        nearest_next = None
        min_distance = float('inf')
        
        for next_node in unvisited:
            _, dist = find_shortest_path_dijkstra(graph, current, next_node, local_active_hazards)
            if dist < min_distance:
                min_distance = dist
                nearest_next = next_node
                
        if nearest_next is None:
            break
            
        unvisited.remove(nearest_next)
        optimized_path.append(nearest_next)
        total_tsp_distance += min_distance
        current = nearest_next
        
    return optimized_path, total_tsp_distance


# FEATURE 4: BFS Module via Deque Library (FIFO Radius Search for Near Me Nodes)
def find_nearest_utility_bfs(graph, start, target_type):
    search_queue = deque([start])
    visited_nodes = {start}
    
    while search_queue:
        current_node = search_queue.popleft()
        if target_type.lower() in current_node.lower():
            return current_node
        for neighbor, _ in graph.get(current_node, []):
            if neighbor not in visited_nodes:
                visited_nodes.add(neighbor)
                search_queue.append(neighbor)
    return None


# FEATURE 5: Live Weather API Layer (Outbound Handshake via Requests.get)
def get_live_weather(city_name="Hatkalangale"):
    print("📡 Attempting to connect to online Weather Server...")
    try:
        url = f"https://wttr.in{city_name}?format=j1"
        response = requests.get(url, timeout=4)
        if response.status_code == 200:
            # FEATURE 12: JSON-Safe Guard avoiding bad response strings or HTML code crashes
            weather_data = response.json()
            current_condition = weather_data['current_condition']['weatherDesc']['value'].lower()
            print("🌐 [ONLINE MODE]: Internet connection active. Live weather fetched successfully.")
            print(f"🌦️ Real-time Weather Report from Internet: {current_condition.upper()}")
            
            if "rain" in current_condition or "shower" in current_condition or "drizzle" in current_condition:
                return "rainy"
            elif "fog" in current_condition or "mist" in current_condition or "haze" in current_condition:
                return "foggy"
            elif "clear" in current_condition or "sunny" in current_condition:
                return "summer"
    except (requests.exceptions.RequestException, ValueError, json.JSONDecodeError):
        # SYSTEM BACKUP: Offline Fallback Local Sensors Conditional Loop System
        print("\n⚠️ [OFFLINE MODE ACTIVE]: Internet network failed, API Server timeout or Bad Gateway JSON Error!")
        print("📴 Activating local vehicle temperature sensors and pre-saved graph cache...")
        simulated_local_sensor_temp = 24
        if simulated_local_sensor_temp > 35:
            return "summer"
        elif simulated_local_sensor_temp < 15:
            return "foggy"
        else:
            return "normal"
    return "normal"


# FEATURE 6: Crowdsourced Anomaly Creator (Requests.post Layer) & Status Modifier (Requests.put Layer)
def upload_road_hazard_post(vehicle_id, location, hazard_type):
    print(f"📡 [POST REQUEST] Vehicle-{vehicle_id} uploading new anomaly to cloud server...")
    cloud_server_database["global_traffic_hazards"][location] = hazard_type
    print("✅ [SERVER RESPONSE 201]: Hazard successfully created on central dashboard.")

def update_road_hazard_put(vehicle_id, location, updated_status):
    print(f"🔄 [PUT REQUEST] Vehicle-{vehicle_id} modifying active cloud database...")
    if location in cloud_server_database["global_traffic_hazards"]:
        cloud_server_database["global_traffic_hazards"][location] = updated_status
        print(f"✅ [SERVER RESPONSE 200]: Server resource successfully updated.")
# =======================================================
# FEATURE 7: Emergency EV Battery Saver (Cloud-Aware Target Overriding Interface)
# =======================================================
def evaluate_ev_emergency_telemetry(battery_percentage, current_destination, local_active_hazards):
    ev_alert = ""
    target_override = current_destination
    
    if battery_percentage < 15:
        ev_alert = "🚨 EMERGENCY EV CRITICAL BATTERY! Eco-restricted LIMP mode active."
        
        # Cloud-Aware Validation: Cross-reference charging points with remote live database
        preferred_charging_spot = "Petrol_Pump"
        if preferred_charging_spot in local_active_hazards and "avoid" in local_active_hazards[preferred_charging_spot].lower():
            print(f"⚠️ [EV GRID CRITICAL ALERT]: Central Server reports '{preferred_charging_spot}' is BLOCKED/ACCIDENT! Dynamic Switch initiated...")
            target_override = "Charging_Station" 
        else:
            target_override = preferred_charging_spot
            
        ev_alert += f" Dynamically locked safe charging grid at: '{target_override}'"
    else:
        ev_alert = "🔋 EV Battery parameters stable. Performance mode active."
        
    return {"ev_status": ev_alert, "final_target": target_override}


# =======================================================
# FEATURE 8: Analytical Driver Quality Grading Calculator
# =======================================================
def calculate_driver_score(weather, driving_speed, improper_tyre):
    score = 100
    if weather == "foggy" and driving_speed > 40: score -= 25  
    if weather == "rainy" and driving_speed > 60: score -= 15  
    if improper_tyre: score -= 10  
    return max(0, score)


# =======================================================
# FEATURE 11: Live Database Telemetry Fetching (Requests.get Layer)
# =======================================================
def fetch_active_cloud_hazards_get():
    print("📡 [GET REQUEST] Vehicle downloading active road hazard matrix from central server...")
    return cloud_server_database["global_traffic_hazards"]


# =======================================================
# ADDITIONAL SUPPORTING ARCHITECTURE: ADAS DASHBOARD LOGIC
# =======================================================
def check_safety_recommendations(weather, driving_speed):
    recommendation = "✅ Standard safe driving parameters applied."
    audio_alert_text = ""
    engine_heat = 40  
    tyre_pressure = 32  
    if weather == "summer":
        engine_heat = 40 + (driving_speed * 0.5)
        tyre_pressure = 32 + (driving_speed * 0.1)
        if engine_heat > 75:
            recommendation = "⚠️ HIGH ENGINE HEAT! Take a break immediately."
            audio_alert_text = "Warning. High engine heat detected."
    elif weather == "foggy" and driving_speed > 40:
        recommendation = "⚠️ SPEED VIOLATION IN FOG! Reduce speed."
        audio_alert_text = "Warning. Heavy fog ahead."
    elif weather == "rainy":
        recommendation = "🌧️ Slippery roads alert. Maintain braking distance."
        
    return {"engine_temp_celsius": round(engine_heat, 2), "tyre_pressure_psi": round(tyre_pressure, 2), "system_advice": recommendation, "audio_alert_payload": audio_alert_text}
# =======================================================
# MAIN TEST EXECUTION INTERFACE (TESTING ALL 12+ PREMIUM FEATURES)
# =======================================================
if __name__ == "__main__":
	from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# 1. Initialize FastAPI main application instance
app = FastAPI(title="Google STEP Standard EV Telematics & Cloud Engine")

# 2. Enable CORS configuration loops to allow frontend index.html connection
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 3. Core Live Routing Telematics API Endpoint Gateway
@app.get("/api/navigate")
def get_live_telematics_route(start: str = Query(...), end: str = Query(...)):
        # ADVANCED LOGIC: Extracts clean node names from complex string tokens (Exactly 9 lines)
    if blocked:
        for entry in blocked.split(","):
            if ":" in entry:
                clean_node_name = entry.split(":")[0].strip()
                blocked_nodes_list.append(clean_node_name)
            else:
                blocked_nodes_list.append(entry.strip())

    
    # 4. Runs your exact battery emergency telemetry checker logic
    simulated_battery = 12 
    ev_matrix = evaluate_ev_emergency_telemetry(simulated_battery, end, live_cloud_hazards)
    final_target = ev_matrix['final_target']
    
    # 5. Automatically switches between A* Search and Dijkstra's algorithm based on target node
    if final_target == "Destination":
        cost, optimal_path = find_shortest_path_astar(city_map, start, final_target, live_cloud_hazards)
        algo_used = "A* Search Algorithm (Heuristic Core)"
    else:
        cost, optimal_path = find_shortest_path_dijkstra(city_map, start, final_target, live_cloud_hazards)
        algo_used = "Dijkstra's Algorithm (Min-Heap Array Core)"
        
    # 6. Fetches real-time weather from Hatkanangale using your wttr.in connector
    try:
        live_weather = get_live_weather("Hatkanangale")
    except:
        live_weather = "normal" 
        
    return {
        "status": "success",
        "algorithm_deployed": algo_used,
        "path": optimal_path,
        "total_distance_km": cost,
        "live_weather": live_weather if live_weather in ['summer', 'rainy', 'foggy'] else 'normal'
    }

# 4. Execution Entry Gate initializing local web thread on Port 8000
if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", )
