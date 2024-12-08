#include <iostream>
#include <fstream>
#include <string>

using namespace std;

#define INF 9999999  // Use a large value as infinity

// Travel_time struct holds the destination and time (weight)
struct Travel_time {
    string destination;
    Travel_time* next;
    int time;
    Travel_time(const string& e, int t, Travel_time* n = NULL)
        : destination(e), time(t), next(n) {}
};

// IntersectionNode represents a single intersection with a list of outgoing edges
struct IntersectionNode {
    Travel_time* edge;
    string name;
    IntersectionNode* next;
    IntersectionNode(const string& n, IntersectionNode* ne = NULL, Travel_time* e = NULL)
        : name(n), next(ne), edge(e) {}
};
struct RoadSignals {
    string Road;
    int SignalTime;
    int VehicleCount;
    RoadSignals* next;

    bool operator< (RoadSignals& a) {
        return this->SignalTime < a.SignalTime;
    }

    RoadSignals(string r, int t) : Road(r), SignalTime(t), VehicleCount(0), next(nullptr) {}
};

struct StackNode {
    string from;
    string to;
    StackNode* next;
    StackNode(const string& f, const string& t, StackNode* n = NULL)
        : from(f), to(t), next(n) {}
};

struct HashNode {
    char start;   // Start intersection
    char end;     // End intersection
    int vehicleCount;
    HashNode* next;

    bool operator==(const HashNode& obj) const {
        return (this->start == obj.start && this->end == obj.end);
    }

    HashNode(char s, char e) : start(s), end(e), vehicleCount(1), next(NULL) {}
};


// RoadNetwork class that manages intersections and roads between them
class RoadNetwork {
private:

    void addTravelTime(IntersectionNode* node, const string& to, int t) {
        Travel_time* newNode = new Travel_time(to, t);
        newNode->next = node->edge;
        node->edge = newNode;
    }

    IntersectionNode* findIntersection(const string& n) {
        IntersectionNode* temp = head;
        while (temp != NULL) {
            if (temp->name == n) {
                return temp;
            }
            temp = temp->next;
        }
        IntersectionNode* newNode = new IntersectionNode(n);
        newNode->next = head;
        head = newNode;
        return newNode;
    }

    void removeTravelTime(IntersectionNode* node, const string& to) {
        Travel_time* temp = node->edge;
        Travel_time* prev = NULL;
        while (temp != NULL) {
            if (temp->destination == to) {
                if (prev == NULL) {
                    node->edge = temp->next;
                } else {
                    prev->next = temp->next;
                }
                delete temp;
                return;
            }
            prev = temp;
            temp = temp->next;
        }
    }
    // Find all paths with DFS and weight calculation
    void findAllPathsDFS(const string& start, const string& destination, bool visited[], string path[], int& pathIndex, string allPaths[], int allWeights[], int weight) {
        visited[start[0] - 'A'] = true;  // Mark current node as visited
        path[pathIndex] = start;          // Add to current path
        pathIndex++;

        if (start == destination) {
            string tempPath = "";
            for (int i = 0; i < pathIndex; i++) {
                tempPath += path[i];
                if (i != pathIndex - 1) {
                    tempPath += " -> ";
                }
            }
            allPaths[pathIndex - 1] = tempPath;
            allWeights[pathIndex - 1] = weight;  // Store the total weight of this path
        } else {
            IntersectionNode* node = head;
            while (node != NULL) {
                if (node->name == start) {
                    Travel_time* edge = node->edge;
                    while (edge != NULL) {
                        if (!visited[edge->destination[0] - 'A']) {
                            weight += edge->time;  // Add the weight of the edge
                            findAllPathsDFS(edge->destination, destination, visited, path, pathIndex, allPaths, allWeights, weight);
                            weight -= edge->time;  // Backtrack by removing the weight
                        }
                        edge = edge->next;
                    }
                }
                node = node->next;
            }
        }

        pathIndex--;  // Backtrack
        visited[start[0] - 'A'] = false; // Unmark node
    }
public:
    IntersectionNode* head;
    RoadNetwork() : head(NULL) {}

    void addTravelTime(const string& from, const string& to, int t) {
        IntersectionNode* fromNode = findIntersection(from);
        IntersectionNode* toNode = findIntersection(to);
        addTravelTime(fromNode, to, t);
        addTravelTime(toNode, from, t); // Since roads are bi-directional
    }

    void removeIntersection(const string& n) {
        IntersectionNode* temp = head;
        IntersectionNode* prev = NULL;
        while (temp != NULL) {
            if (temp->name == n) {
                if (prev == NULL) {
                    head = temp->next;
                } else {
                    prev->next = temp->next;
                }
                delete temp;
                return;
            }
            prev = temp;
            temp = temp->next;
        }
    }

    void removeTravelTime(const string& from, const string& to) {
        IntersectionNode* fromNode = findIntersection(from);
        IntersectionNode* toNode = findIntersection(to);
        removeTravelTime(fromNode, to);
        removeTravelTime(toNode, from); // Since roads are bi-directional
    }

    void print() 
    {
        IntersectionNode* temp = head;
        while (temp != NULL) {
            cout << temp->name << " -> ";
            Travel_time* temp2 = temp->edge;
            while (temp2 != NULL) {
                cout << " (" << temp2->destination << " , " << temp2->time << ")  ";
                temp2 = temp2->next;
            }
            cout << endl;
            temp = temp->next;
        }
    }

    // Dijkstra's Algorithm to find the shortest path from a start node to a destination
    void dijkstra(const string& start, const string& destination) {
        // Maximum number of intersections, assuming no more than 100 nodes.
        const int MAX_NODES = 100;
        string nodeNames[MAX_NODES];  // Array of node names
        int dist[MAX_NODES];  // Distance array
        int parent[MAX_NODES];  // Parent array to reconstruct the path
        bool visited[MAX_NODES];  // Visited array

        int nodeCount = 0;
        IntersectionNode* temp = head;

        // Initialize the nodeNames, distance, parent, and visited arrays
        while (temp != NULL) {
            nodeNames[nodeCount] = temp->name;
            dist[nodeCount] = INF;
            parent[nodeCount] = -1;
            visited[nodeCount] = false;
            nodeCount++;
            temp = temp->next;
        }

        int startIndex = -1, destIndex = -1;
        for (int i = 0; i < nodeCount; i++) {
            if (nodeNames[i] == start) startIndex = i;
            if (nodeNames[i] == destination) destIndex = i;
        }

        if (startIndex == -1 || destIndex == -1) {
            cout << "Start or destination node not found!" << endl;
            return;
        }

        dist[startIndex] = 0;  // Distance to the start node is 0

        // Dijkstra's algorithm
        for (int i = 0; i < nodeCount; i++) {
            // Find the unvisited node with the smallest distance
            int u = -1;
            int minDist = INF;
            for (int j = 0; j < nodeCount; j++) {
                if (!visited[j] && dist[j] < minDist) {
                    u = j;
                    minDist = dist[j];
                }
            }

            if (u == -1) break;  // No more reachable nodes

            visited[u] = true;

            // Update the distance to neighboring nodes
            IntersectionNode* uNode = head;
            for (int j = 0; j < u; j++) uNode = uNode->next;

            Travel_time* edge = uNode->edge;
            while (edge != NULL) {
                int v = -1;
                for (int k = 0; k < nodeCount; k++) {
                    if (nodeNames[k] == edge->destination) {
                        v = k;
                        break;
                    }
                }

                if (v != -1 && dist[u] + edge->time < dist[v]) {
                    dist[v] = dist[u] + edge->time;
                    parent[v] = u;
                }

                edge = edge->next;
            }
        }

        // Reconstruct the shortest path
        if (dist[destIndex] == INF) {
            cout << "No path found!" << endl;
            return;
        }

        cout << "Shortest path from " << start << " to " << destination << " is: ";
        int currentNode = destIndex;
        string path = destination;
        while (parent[currentNode] != -1) {
            currentNode = parent[currentNode];
            path = nodeNames[currentNode] + " -> " + path;
        }
        
        // Output the shortest path
        cout << path << endl;

        // Output the weight (total travel time)
        cout << "Total travel time: " << dist[destIndex] << " units" << endl;
    }

    // Function to remove roads (based on road closure data)
    void removeRoads(const string& from, const string& to) {
        removeTravelTime(from, to);
    }
    void displayAllPaths(const string& start, const string& destination) {
            // Array for visited nodes and paths
        bool visited[26] = {false};  // Assuming there are 4 nodes (A, B, C, D)
        string path[1000];             // Path for storing the current route
        int pathIndex = 0;          // Index to track the path
        string allPaths[1000];        // Array to store all paths
        int allWeights[1000];         // Array to store the weights of all paths

        int weight = 0;             // Variable to track the weight (travel time)
        // Find all paths from A to D
        findAllPathsDFS(start, destination, visited, path, pathIndex, allPaths, allWeights, weight);

        // Display all paths and their weights
        for (int i = 0; i < 10; i++) {
            if (!allPaths[i].empty()) {
                cout << allPaths[i] << " | Weight: " << allWeights[i] << endl;
            }
        }
    }
    IntersectionNode* getIntersection(const string& n) {
        IntersectionNode* temp = head;
        while (temp != NULL) {
            if (temp->name == n) {
                return temp;
            }
            temp = temp->next;
        }
        return NULL;
    }
    ~RoadNetwork() {
        while (head) {
            IntersectionNode* temp = head;
            head = head->next;
            Travel_time* edge = temp->edge;
            while (edge) {
                Travel_time* tempEdge = edge;
                edge = edge->next;
                delete tempEdge;
            }
            delete temp;
        }
    }
};
// Custom stack class for storing blocked roads
class BlockedRoadStack {
private:
    StackNode* top;
public:
    BlockedRoadStack() : top(NULL) {}

    void push(const string& from, const string& to) {
        StackNode* newNode = new StackNode(from, to);
        newNode->next = top;
        top = newNode;
    }

    void pop() {
        if (top != NULL) {
            StackNode* temp = top;
            top = top->next;
            delete temp;
        }
    }

    bool isEmpty() const {
        return top == NULL;
    }

    void print() const {
        StackNode* temp = top;
        while (temp != NULL) {
            cout << "Blocked Road: " << temp->from << " -> " << temp->to << endl;
            temp = temp->next;
        }
    }
};


class TrafficSignals {
private:
    RoadSignals* head;

public:
    // Constructor
    TrafficSignals() : head(nullptr) {}

    // Insert a new RoadSignal into the priority queue
    void insert(string road, int signalTime) {
        RoadSignals* newNode = new RoadSignals(road, signalTime);

        // If the queue is empty or the new node has the highest priority
        if (!head || signalTime < head->SignalTime) {
            newNode->next = head;
            head = newNode;
            return;
        }

        // Find the correct position to insert the new node
        RoadSignals* temp = head;
        while (temp->next && temp->next->SignalTime <= signalTime) {
            temp = temp->next;
        }

        newNode->next = temp->next;
        temp->next = newNode;
    }

    // Remove the highest priority element (head of the list)
    string dequeue() {
        if (!head) {
            cout << "Traffic signal queue is empty!" << endl;
            return "";
        }

        RoadSignals* temp = head;
        string road = head->Road;
        head = head->next;
        delete temp;
        return road;
    }

    // Peek at the highest priority element without removing it
    string peek() {
        if (!head) {
            cout << "Traffic signal queue is empty!" << endl;
            return "";
        }
        return head->Road;
    }

    // Check if the priority queue is empty
    bool isEmpty() {
        return head == nullptr;
    }

    // Display all signals in the queue
    void display() {
        if (!head) {
            cout << "Traffic signal queue is empty!" << endl;
            return;
        }

        RoadSignals* temp = head;
        while (temp) {
            cout << "Intersection: " << temp->Road 
                 << ", Green Time: " << temp->SignalTime 
                 << ", Vehicles: " << temp->VehicleCount << endl;
            temp = temp->next;
        }
    }
    int* getGreenTime()
    {
        if (!head) {
            cout << "Traffic signal queue is empty!" << endl;
            return nullptr;
        }
        else
        {
            int* greenTime = new int[26];
            RoadSignals* temp = head;
            while (temp) {
                greenTime[temp->Road[0] - 'A'] = temp->SignalTime;
                temp = temp->next;
            }
            return greenTime;
        }
    }
    void UpdateVehiclesCount(string r, int v) {
        RoadSignals* temp = head;

        while (temp) {
            if (temp->Road == r) {
                temp->VehicleCount = v;
                return;
            }
            temp = temp->next;
        }

        cerr << "Error: Intersection " << r << " not found in the traffic signals list." << endl;
    }

    void calculateNewGreenTimes() {
        RoadSignals* temp = head;
        while (temp) {
            if (temp->VehicleCount > 1) {
                temp->SignalTime += 5 * (temp->VehicleCount - 1);
            }
            temp = temp->next;
        }
    }
    void setGreenTime(string intersection, int time, int index) {
        if (isEmpty()) {
            cout << "Traffic signal queue is empty!" << endl;
            return;
        }
        else
        {
            RoadSignals* temp = head;
            while (temp) {
                if (temp->Road == intersection) {
                    temp->SignalTime = time;
                    break;
                }
                temp = temp->next;
            }
        }
    }
    void resetGreenTime(string intersection, int normalTime, int index) {
        if (isEmpty()) {
            cout << "Traffic signal queue is empty!" << endl;
            return;
        }
        else
        {
            RoadSignals* temp = head;
            while (temp) {
                if (temp->Road == intersection) {
                    temp->SignalTime = normalTime;
                    break;
                }
                temp = temp->next;
            }
        }
    }
    ~TrafficSignals() {
        while (head) {
            RoadSignals* temp = head;
            head = head->next;
            delete temp;
        }
    }
};




class EmergencyVehicleHandling {
public:
    RoadNetwork* roadNetwork;
    TrafficSignals* trafficSignals;
    string allPaths[26];  // Array to store paths (max 26 paths)
    int allWeights[1000]; // Array to store the weights for each path

    EmergencyVehicleHandling(RoadNetwork* network, TrafficSignals* signals) {
        roadNetwork = network;
        trafficSignals = signals;
    }

    // Simple heuristic function (returns 0 for simplicity, can be customized)
    int heuristic(const string& current, const string& goal) {
        return 0;  // Modify this for an actual heuristic if needed
    }

    void handleEmergencyVehicle(const string& start, const string& destination) {
        const int MAX_NODES = 100;
        string nodeNames[MAX_NODES];
        int gCost[MAX_NODES];     // Cost from start to current node
        int fCost[MAX_NODES];     // Estimated total cost (g + h)
        bool visited[MAX_NODES];  // Track visited nodes
        int parent[MAX_NODES];    // To reconstruct the path

        IntersectionNode* nodes[MAX_NODES];
        int nodeCount = 0;
        IntersectionNode* temp = roadNetwork->head;

        // Initialize the node arrays
        while (temp != NULL) {
            nodes[nodeCount] = temp;
            nodeNames[nodeCount] = temp->name;
            gCost[nodeCount] = INF;
            fCost[nodeCount] = INF;
            visited[nodeCount] = false;
            parent[nodeCount] = -1;
            nodeCount++;
            temp = temp->next;
        }

        // Find start and destination indices
        int startIndex = -1, destIndex = -1;
        for (int i = 0; i < nodeCount; i++) {
            if (nodeNames[i] == start) startIndex = i;
            if (nodeNames[i] == destination) destIndex = i;
        }

        if (startIndex == -1 || destIndex == -1) {
            cout << "Start or destination node not found!" << endl;
            return;
        }

        // Initialize the priority queue (simple implementation)
        int openSet[MAX_NODES];
        int openSetSize = 1;
        openSet[0] = startIndex;

        gCost[startIndex] = 0;
        fCost[startIndex] = heuristic(start, destination);

        while (openSetSize > 0) {
            // Find node with the smallest fCost in the open set
            int currentIndex = openSet[0];
            int currentMinF = fCost[currentIndex];
            int minIndex = 0;

            for (int i = 1; i < openSetSize; i++) {
                if (fCost[openSet[i]] < currentMinF) {
                    currentIndex = openSet[i];
                    currentMinF = fCost[currentIndex];
                    minIndex = i;
                }
            }

            // Remove the current node from the open set
            openSetSize--;
            for (int i = minIndex; i < openSetSize; i++) {
                openSet[i] = openSet[i + 1];
            }
            visited[currentIndex] = true;

            // If we reached the destination
            if (currentIndex == destIndex) {
                break;
            }

            // Get the current node's neighbors
            IntersectionNode* currentNode = nodes[currentIndex];
            Travel_time* edge = currentNode->edge;

            while (edge != NULL) {
                int neighborIndex = -1;
                for (int i = 0; i < nodeCount; i++) {
                    if (nodeNames[i] == edge->destination) {
                        neighborIndex = i;
                        break;
                    }
                }

                if (neighborIndex != -1 && !visited[neighborIndex]) {
                    int tentativeGCost = gCost[currentIndex] + edge->time;

                    if (tentativeGCost < gCost[neighborIndex]) {
                        gCost[neighborIndex] = tentativeGCost;
                        fCost[neighborIndex] = gCost[neighborIndex] + heuristic(edge->destination, destination);
                        parent[neighborIndex] = currentIndex;

                        // Add neighbor to the open set if not already present
                        bool inOpenSet = false;
                        for (int i = 0; i < openSetSize; i++) {
                            if (openSet[i] == neighborIndex) {
                                inOpenSet = true;
                                break;
                            }
                        }
                        if (!inOpenSet) {
                            openSet[openSetSize++] = neighborIndex;
                        }
                    }
                }
                edge = edge->next;
            }
        }

        // Reconstruct and display the path
        if (gCost[destIndex] == INF) {
            cout << "No path found for the emergency vehicle!" << endl;
            return;
        }

        cout << "Emergency vehicle path from " << start << " to " << destination << " is: ";
        int path[MAX_NODES];
        int pathLength = 0;
        int current = destIndex;

        while (current != -1) {
            path[pathLength++] = current;
            current = parent[current];
        }

        for (int i = pathLength - 1; i >= 0; i--) {
            cout << nodeNames[path[i]];
            if (i != 0) cout << " -> ";
        }
        cout << endl;

        cout << "Total travel time: " << gCost[destIndex] << " units" << endl;

        // Override traffic signals for the emergency vehicle's path
        for (int i = pathLength - 1; i >= 0; i--) {
            string intersection = nodeNames[path[i]];
            trafficSignals->setGreenTime(intersection, 0, i);
            cout << "Override green time at intersection " << intersection << " to 0." << endl;
        }

        // Restore normal green times after the emergency vehicle passes
        for (int i = pathLength - 1; i >= 0; i--) {
            string intersection = nodeNames[path[i]];
            int normalGreenTime = 30;  // Assuming the normal green time is 30
            trafficSignals->resetGreenTime(intersection, normalGreenTime, i);
            cout << "Restored green time at intersection " << intersection << " to " << normalGreenTime << "." << endl;
        }
    }
    ~EmergencyVehicleHandling() 
    {
        roadNetwork = nullptr;
        trafficSignals = nullptr;
    }
};

class VehicleHashTable {
private:
    HashNode** table;
    int capacity;
    int size;
    const float loadFactor = 0.75;

    // Hash function to calculate index based on start and end
    int hash(char start, char end) {
        return ((start - 'A') * 26 + (end - 'A')) % capacity;
    }

    // Resize the hash table when load factor exceeds the threshold
    void resize() {
        int newCapacity = capacity * 2;
        HashNode** newTable = new HashNode*[newCapacity]();

        for (int i = 0; i < capacity; i++) {
            HashNode* node = table[i];
            while (node) {
                int newIndex = hash(node->start, node->end) % newCapacity;
                HashNode* nextNode = node->next;

                // Reinsert node into new table
                node->next = newTable[newIndex];
                newTable[newIndex] = node;

                node = nextNode;
            }
        }

        delete[] table;
        table = newTable;
        capacity = newCapacity;
    }

public:
    VehicleHashTable(int initialCapacity = 10) : capacity(initialCapacity), size(0) {
        table = new HashNode*[capacity]();
    }

    ~VehicleHashTable() {
        for (int i = 0; i < capacity; i++) {
            HashNode* node = table[i];
            while (node) {
                HashNode* temp = node;
                node = node->next;
                delete temp;
            }
        }
        delete[] table;
    }

    // Insert or update vehicle count for a road
    void insert(char start, char end) {
        int index = hash(start, end) % capacity;
        HashNode* node = table[index];
        while (node) {
            if (node->start == start && node->end == end) {
                node->vehicleCount++;
                return;
            }
            node = node->next;
        }

        // Insert new node
        node = new HashNode(start, end);
        node->next = table[index];
        table[index] = node;
        size++;

        if (size > capacity * loadFactor) {
            resize();
        }
    }

    // Display all road segments and their vehicle counts
    void display() {
        for (int i = 0; i < capacity; i++) {
            HashNode* node = table[i];
            while (node) {
                cout << "Road: (" << node->start << " -> " << node->end << "), Vehicles: " << node->vehicleCount << endl;
                node = node->next;
            }
        }
    }
    
    void displayCongestion() {
        const int numIntersections = 26; // For intersections 'A' to 'Z'
        int routeCounts[numIntersections][numIntersections] = {0}; // 2D array to store counts

        // Traverse all intersections as start and end points
        for (char start = 'A'; start <= 'Z'; ++start) {
            for (char end = 'A'; end <= 'Z'; ++end) {
                if (start == end) continue; // Skip if start and end are the same
                
                int totalVehicles = 0; // Initialize vehicle count for the route

                // Traverse through the hash table
                for (int i = 0; i < capacity; ++i) {
                    HashNode* node = table[i];
                    while (node != nullptr) {
                        // Check if the current route is part of the path between start and end
                        if ((node->start >= start && node->end <= end) || 
                            (node->start <= end && node->end >= start)) {
                            totalVehicles += node->vehicleCount;
                        }
                        node = node->next;
                    }
                }

                // Store the cumulative vehicle count in the 2D array
                if (totalVehicles > 0) {
                    routeCounts[start - 'A'][end - 'A'] = totalVehicles;
                }
            }
        }

        // Display the cumulative vehicle counts
        cout << "------ CongestionStatus ------" << endl;
        for (char start = 'A'; start <= 'Z'; ++start) {
            for (char end = 'A'; end <= 'Z'; ++end) {
                if (start != end && routeCounts[start - 'A'][end - 'A'] > 0) {
                    cout << start << " to " << end << " -> Vehicles: " 
                        << routeCounts[start - 'A'][end - 'A'] << endl;
                }
            }
        }
    }
};

// Function to read the road network data from a CSV file
void read_roadNetwork(const string& filename, RoadNetwork& road) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Error: Unable to open the file" << endl;
        return;
    }

    string line;
    getline(file, line); // Skip the first line (header)

    while (getline(file, line)) {
        // Find the positions of the delimiters
        int pos1 = line.find(',');
        int pos2 = line.find_last_of(',');

        // Validate positions
        if (pos1 == -1 || pos2 == -1 || pos1 == pos2) {
            cout << "Error: Invalid line format: " << line << endl;
            continue;
        }

        // Extract fields
        string from = line.substr(0, pos1);
        string to = line.substr(pos1 + 1, pos2 - pos1 - 1);
        string weightStr = line.substr(pos2 + 1);

        // Check if weightStr is a valid number
        bool isValidNumber = true;
        for (char c : weightStr) {
            if (!isdigit(c)) {
                isValidNumber = false;
                break;
            }
        }

        if (isValidNumber) {
            int weight = stoi(weightStr); // Convert the weight to an integer
                     road.addTravelTime(from, to, weight);
        } else {
            cout << "Error: Invalid weight value in line: " << line << endl;
        }
    }

    file.close();
}

// Function to read road closures and remove corresponding roads
void read_roadClosures(const string& filename, RoadNetwork& road, BlockedRoadStack& blockedRoads) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Error: Unable to open the road closure file" << endl;
        return;
    }

    string line;
    getline(file, line); // Skip the first line (header)

    while (getline(file, line)) {
        // Find the positions of the delimiters
        int pos1 = line.find(',');
        int pos2 = line.find_last_of(',');

        // Validate positions
        if (pos1 == -1 || pos2 == -1 || pos1 == pos2) {
            cout << "Error: Invalid line format: " << line << endl;
            continue;
        }

        // Extract fields
        string from = line.substr(0, pos1);
        string to = line.substr(pos1 + 1, pos2 - pos1 - 1);

        // Push the blocked road to the stack
        string status = line.substr(pos2+1);
        if(status == "Blocked")
        {
            blockedRoads.push(from, to);
        }
        // Remove the road
        if(status == "Clear")
        {
            road.removeRoads(from, to);
            road.removeRoads(to, from);
        }  // Since roads are bi-directional
    }

    file.close();
}

void read_trafficSignals(const std::string& trafficFile, const std::string& vehicleFile, TrafficSignals& obj) {
    ifstream trafficStream(trafficFile);
    ifstream vehicleStream(vehicleFile);

    if (!trafficStream.is_open()) {
        cerr << "Error: Could not open traffic signals file " << trafficFile << std::endl;
        return;
    }

    if (!vehicleStream.is_open()) {
        cerr << "Error: Could not open vehicles file " << vehicleFile << std::endl;
        return;
    }

    const int MAX_INTERSECTIONS = 100;
    string roads[MAX_INTERSECTIONS];
    int vehicleCounts[MAX_INTERSECTIONS] = {0};
    int roadCount = 0;

    string line;
    int index = 0;
    while (std::getline(vehicleStream, line)) {
        size_t pos = line.find(',');
        if (pos != std::string::npos) {
            string road = line.substr(pos + 1);
            road = road.substr(0, road.find(','));

            bool found = false;
            for (int i = 0; i < roadCount; ++i) {
                if (roads[i] == road) {
                    vehicleCounts[i]++;
                    found = true;
                    break;
                }
            }

            if (!found && roadCount < MAX_INTERSECTIONS) {
                roads[roadCount] = road;
                vehicleCounts[roadCount] = 1;
                roadCount++;
            }
        }
    }

    vehicleStream.close();

    index = 0;
    while (getline(trafficStream, line)) {
        size_t pos = line.find(',');
        if (pos != std::string::npos) {
            string road = line.substr(0, pos);
            string signalTimeStr = line.substr(pos + 1);

            try {
                int signalTime = std::stoi(signalTimeStr);
                obj.insert(road, signalTime);
            } catch (const std::invalid_argument&) {
                std::cerr << "Error: Invalid data format in line: " << line << std::endl;
            }
        }
    }

    trafficStream.close();

    for (int i = 0; i < roadCount; ++i) {
        obj.UpdateVehiclesCount(roads[i], vehicleCounts[i]);
    }
}

// Function to read vehicle data from a CSV file
void read_vehicles(const string& filename, VehicleHashTable& obj) {
    ifstream file(filename);

    // Check if the file opened successfully
    if (!file.is_open()) {
        cout << "Error: Unable to open the vehicles file" << endl;
        return;
    }

    string line;
    getline(file, line); // Skip the first line (header)

    while (getline(file, line)) {
        int commaPos = -1;
        
        // Find the position of the first comma manually
        for (int i = 0; i < line.length(); i++) {
            if (line[i] == ',') {
                commaPos = i;
                break;
            }
        }

        // Ensure the comma exists and there's enough data after the comma
        if (commaPos == -1 || commaPos + 3 >= line.length()) {
            cerr << "Error: Invalid line format: " << line << endl;
            continue;
        }

        // Extract the vehicle ID (substring before the comma)
        string vehicleID = line.substr(0, commaPos);

        // Extract the start and end nodes (characters after the comma)
        char start = line[commaPos + 1];
        char end = line[commaPos + 3];

        // Insert the start and end nodes into the VehicleHashTable
        obj.insert(start, end);
    }

    file.close();
}
void read_emergencyVehicle(const string& filename, EmergencyVehicleHandling& obj) 
{
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Error: Unable to open the road closure file" << endl;
        return;
    }

    string line;
    getline(file, line); // Skip the first line (header)
    int i = 0;
    while (getline(file, line)) {
        // Find the positions of the delimiters
        int pos1 = line.find(',');
        int pos2 = line.find_last_of(',');

        // Validate positions
        if (pos1 == -1 || pos2 == -1 || pos1 == pos2) {
            cout << "Error: Invalid line format: " << line << endl;
            continue;
        }

        // Extract fields
        string from = line.substr(pos1+1, 1);
        string to = line.substr(pos2 - 1, 1);
        obj.handleEmergencyVehicle(from, to);
    }
    file.close();
}
int main() {
    // Create instances of your objects
    RoadNetwork Orignal_road, road;
    TrafficSignals signals;
    BlockedRoadStack blockedRoads;
    VehicleHashTable vehicleData;

    // Placeholder for reading CSV files
    read_roadNetwork("road_network.csv", Orignal_road);
    read_roadNetwork("road_network.csv", road);
    read_trafficSignals("traffic_signals.csv", "vehicles.csv", signals);
    read_vehicles("vehicles.csv", vehicleData);
    read_roadClosures("road_closures.csv", road, blockedRoads);

    int choice = 0;
    // Create SFML window
    sf::RenderWindow window(sf::VideoMode(800, 600), "Traffic Simulation Menu");

    // Load font for text
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Error loading font\n";
        return -1;
    }

    // Menu options
    sf::Text option1("1. Display city traffic network", font, 30);
    sf::Text option2("2. Display traffic signal status", font, 30);
    sf::Text option3("3. Display congestion status", font, 30);
    sf::Text option4("4. Display blocked roads", font, 30);
    sf::Text option5("5. Handle emergency vehicles Routing", font, 30);
    sf::Text option6("6. Block road due to accident", font, 30);
    sf::Text option7("7. Stimulate vehicle routing", font, 30);
    sf::Text option8("8. Display City Network of Working Roads", font, 30);
    sf::Text option9("9. Handle Emergency Routing for CSV", font, 30);
    sf::Text option10("10. Apply Dijkstra Algorithm", font, 30);
    sf::Text option11("11. Exit Simulation", font, 30);

    // Position menu options
    option1.setPosition(200, 100);
    option2.setPosition(200, 150);
    option3.setPosition(200, 200);
    option4.setPosition(200, 250);
    option5.setPosition(200, 300);
    option6.setPosition(200, 350);
    option7.setPosition(200, 400);
    option8.setPosition(200, 450);
    option9.setPosition(200, 500);
    option10.setPosition(200, 550);
    option11.setPosition(200, 600);

    int selectedOption = 0;
    option1.setFillColor(sf::Color::Red);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed) {
                // Move selection up and down
                if (event.key.code == sf::Keyboard::Up && selectedOption > 0) {
                    selectedOption--;
                } else if (event.key.code == sf::Keyboard::Down && selectedOption < 10) {
                    selectedOption++;
                }

                // Change color based on selection
                option1.setFillColor(selectedOption == 0 ? sf::Color::Red : sf::Color::White);
                option2.setFillColor(selectedOption == 1 ? sf::Color::Red : sf::Color::White);
                option3.setFillColor(selectedOption == 2 ? sf::Color::Red : sf::Color::White);
                option4.setFillColor(selectedOption == 3 ? sf::Color::Red : sf::Color::White);
                option5.setFillColor(selectedOption == 4 ? sf::Color::Red : sf::Color::White);
                option6.setFillColor(selectedOption == 5 ? sf::Color::Red : sf::Color::White);
                option7.setFillColor(selectedOption == 6 ? sf::Color::Red : sf::Color::White);
                option8.setFillColor(selectedOption == 7 ? sf::Color::Red : sf::Color::White);
                option9.setFillColor(selectedOption == 8 ? sf::Color::Red : sf::Color::White);
                option10.setFillColor(selectedOption == 9 ? sf::Color::Red : sf::Color::White);
                option11.setFillColor(selectedOption == 10 ? sf::Color::Red : sf::Color::White);

                // Execute action when Enter is pressed
                if (event.key.code == sf::Keyboard::Enter) {
                    switch (selectedOption) {
                        case 0:
                            std::cout << "Displaying city traffic network...\n";
                            std::cout << "Initial Road Network: " << std::endl;
                            Orignal_road.print();
                            break;
                        case 1:
                            std::cout << "Displaying traffic signal status...\n";
                            signals.display();
                            break;
                        case 2:
                            std::cout << "Displaying congestion status...\n";
                            vehicleData.displayCongestion();
                            break;
                        case 3:
                            std::cout << "Displaying blocked roads...\n";
                            blockedRoads.print();
                            break;
                        case 4: {
                            std::cout << "Handling emergency vehicles...\n";
                            std::cout << "Enter the start and end intersection for emergency vehicle routing: ";
                            char start, end;
                            std::cin >> start >> end;
                            std::cout << "Emergency Vehicle is being Routed...." << std::endl;
                            EmergencyVehicleHandling evHandler(&road, &signals);
                            evHandler.handleEmergencyVehicle(std::string(1, start), std::string(1, end));
                            break;
                        }
                        case 5: {
                            std::cout << "Blocking road due to accident...\n";
                            std::cout << "Enter the road to be blocked (start, end): ";
                            std::string from, to;
                            std::cin >> from >> to;
                            road.removeRoads(from, to);
                            road.removeRoads(to, from);
                            blockedRoads.push(from, to);
                            break;
                        }
                        case 6: {
                            std::cout << "Stimulating vehicle routing...\n";
                            std::cout << "Enter the start and end intersection for vehicle routing: ";
                            char start, end;
                            std::cin >> start >> end;
                            std::cout << "All Possible Paths from " << start << " to " << end << std::endl;
                            road.displayAllPaths(std::string(1, start), std::string(1, end));
                            break;
                        }
                        case 7: {
                            std::cout << "Working Road Network: " << std::endl;
                            road.print();
                            break;
                        }
                        case 8: {
                            EmergencyVehicleHandling evHandler(&road, &signals);
                            read_emergencyVehicle("emergency_vehicles.csv", evHandler);
                            break;
                        }
                        case 9: {
                            std::cout << "Enter Start and End Intersection to apply Dijkstra's Algorithm: ";
                            char start, end;
                            std::cin >> start >> end;
                            road.dijkstra(std::string(1, start), std::string(1, end));
                            break;
                        }
                        case 10:
                            std::cout << "Exiting simulation...\n";
                            window.close(); // Exit the program
                            break;
                        default:
                            std::cout << "Invalid option\n";
                            break;
                    }
                }
            }

            // Clear the window and redraw the options
            window.clear();
            window.draw(option1);
            window.draw(option2);
            window.draw(option3);
            window.draw(option4);
            window.draw(option5);
            window.draw(option6);
            window.draw(option7);
            window.draw(option8);
            window.draw(option9);
            window.draw(option10);
            window.draw(option11);
            window.display();
        }
    }

    return 0;
}
