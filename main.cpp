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
    RoadSignals* next;

    bool operator< (RoadSignals& a) {
        return this->SignalTime < a.SignalTime;
    }

    RoadSignals(string r, int t) : Road(r), SignalTime(t), next(nullptr) {}
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
    IntersectionNode* head;

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
    void findAllPathsDFS(const string& start, const string& destination, bool visited[], string path[], int& pathIndex, string allPaths[]) {
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
        } else {
            IntersectionNode* node = head;
            while (node != NULL) {
                if (node->name == start) {
                    Travel_time* edge = node->edge;
                    while (edge != NULL) {
                        if (!visited[edge->destination[0] - 'A']) {
                            findAllPathsDFS(edge->destination, destination, visited, path, pathIndex, allPaths);
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
    void findPaths(const string& start, const string& destination) {
        bool visited[26] = {false};
        string path[100];
        string allPaths[100];
        int pathIndex = 0;
       
    findAllPathsDFS(start, destination, visited, path, pathIndex, allPaths);

        cout << "All possible paths from " << start << " to " << destination << " are:" << endl;
        for (int i = 0; i < 100; i++) {
            if (!allPaths[i].empty()) {
                cout << allPaths[i] << endl;
            }
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
            cout << "InterSection : " << temp->Road << ", Green Time: " << temp->SignalTime << endl;
            temp = temp->next;
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
void read_trafficSignals(const string& filename, TrafficSignals& obj) {
    ifstream file(filename);

    // Check if file opened successfully
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return;
    }

    string line;
    while (getline(file, line)) {
        int commaPos = -1;

        // Find the position of the comma
        for (int i = 0; i < line.length(); i++) {
            if (line[i] == ',') {
                commaPos = i;
                break;
            }
        }

        // Validate the format
        if (commaPos == -1) {
            cerr << "Error: Invalid line format: " << line << endl;
            continue;
        }

        // Extract the road name and signal time
        string road = line.substr(0, commaPos);
        string signalTimeStr = line.substr(commaPos + 1);

        // Validate signal time
        bool valid = true;
        for (int i = 0; i < signalTimeStr.length(); i++) {
            if (!isdigit(signalTimeStr[i])) {
                valid = false;
                break;
            }
        }

        if (valid) {
            int signalTime = 0;

            // Convert signalTimeStr to an integer
            for (int i = 0; i < signalTimeStr.length(); i++) {
                signalTime = signalTime * 10 + (signalTimeStr[i] - '0');
            }

            // Insert into the TrafficSignals object
            obj.insert(road, signalTime);
        } else {
            cerr << "Error: Invalid signal time in line: " << line << endl;
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
        road.removeRoads(from, to);
        road.removeRoads(to, from);  // Since roads are bi-directional
    }

    file.close();
}

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

int main() {
    RoadNetwork road;
    TrafficSignals signals;
    BlockedRoadStack blockedRoads;
    VehicleHashTable vehicleData;

    // Read road network data from CSV file
    read_roadNetwork("road_network.csv", road);
    
    // Print the initial road network
    cout << "Initial Road Network: " << endl;
    road.print();
    

    // Example usage: Calculate all possible paths between intersections
    // Print all possible paths
    string start = "A";
    string destination = "F";
    road.findPaths(start, destination);

    // Read road closures from a file and update the road network
    read_roadClosures("road_closures.csv", road, blockedRoads);

    // Print the updated road network after road closures
    cout << "\nUpdated Road Network (After Road Closures): " << endl;
    road.print();

    // Calculate the shortest path after road closures
    cout << "\nShortest Path (After Dijkstra's with Road Closures): " << endl;
    road.dijkstra("A", "F");

    read_trafficSignals("traffic_signals.csv", signals );
    signals.display();

    blockedRoads.print();

    // Read vehicle data from a CSV file
    read_vehicles("vehicles.csv", vehicleData);

    // Display the vehicle data
    cout << "\nVehicle Data: " << endl;
    vehicleData.display();

    return 0;
}
