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

    void print() {
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
void read_roadClosures(const string& filename, RoadNetwork& road) {
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

        // Remove the road
        road.removeRoads(from, to);
        road.removeRoads(to, from);  // Since roads are bi-directional
    }

    file.close();
}

int main() {
    RoadNetwork road;
    
    // Read road network data from CSV file
    read_roadNetwork("road_network.csv", road);
    
    // Print the initial road network
    cout << "Initial Road Network: " << endl;
    road.print();

    // Example usage: Calculate all possible paths between intersections
    cout << "\nAll Possible Paths (Before Dijkstra's): " << endl;
    road.dijkstra("A", "F");

    // Read road closures from a file and update the road network
    read_roadClosures("road_closures.csv", road);

    // Print the updated road network after road closures
    cout << "\nUpdated Road Network (After Road Closures): " << endl;
    road.print();

    // Calculate the shortest path after road closures
    cout << "\nShortest Path (After Dijkstra's with Road Closures): " << endl;
    road.dijkstra("A", "F");

    return 0;
}
