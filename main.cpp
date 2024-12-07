#include<iostream>
#include<fstream>
#include<string>
using namespace std;
struct Travel_time
{
    string destination;
    Travel_time* next;
    int time;
    Travel_time(const string& e, int t, Travel_time* n = NULL)
    {
        destination = e;
        time = t;
        next = n;
    }
};
struct IntersectionNode
{
    Travel_time* edge;
    string name;
    IntersectionNode* next;
    IntersectionNode(const string& n, IntersectionNode* ne = NULL, Travel_time* e = NULL)
    {
        name = n;
        next = ne;
        edge = e;
    }
    
};
class RoadNetwork
{
    private:
        IntersectionNode* head;
        void addTravelTime(IntersectionNode* node, const string& to, int t)
        {
            // Add the travel time to the linked list
            Travel_time* newNode = new Travel_time(to, t);
            newNode->next = node->edge;
            node->edge = newNode;
        }
        IntersectionNode* findIntersection(const string& n)
        {
            // Search for the intersection
            IntersectionNode* temp = head;
            while(temp != NULL)
            {
                if(temp->name == n)
                {
                    return temp;
                }
                temp = temp->next;
            }
            // If the intersection is not found, create a new one
            IntersectionNode* newNode = new IntersectionNode(n);
            newNode->next = head;
            head = newNode;
            return newNode;
        }
        void removeTravelTime(IntersectionNode* node, const string& to)
        {
            // Remove the travel time from the linked list
            Travel_time* temp = node->edge;
            Travel_time* prev = NULL;
            while(temp != NULL)
            {
                if(temp->destination == to)
                {
                    if(prev == NULL)
                    {
                        node->edge = temp->next;
                    }
                    else
                    {
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
        RoadNetwork()
        {
            head = NULL;
        }
        void addTravelTime(const string& from, const string& to, int t)
        {
            IntersectionNode* fromNode = findIntersection(from);
            IntersectionNode* toNode = findIntersection(to);
            addTravelTime(fromNode, to, t);
            addTravelTime(toNode, from, t);
        }
        void removeIntersection(const string& n)
        {
            // Remove the intersection
            IntersectionNode* temp = head;
            IntersectionNode* prev = NULL;
            while(temp != NULL)
            {
                if(temp->name == n)
                {
                    if(prev == NULL)
                    {
                        head = temp->next;
                    }
                    else
                    {
                        prev->next = temp->next;
                    }
                    delete temp;
                    return;
                }
                prev = temp;
                temp = temp->next;
            }
        }
        void removeTravelTime(const string& from, const string& to)
        {
            IntersectionNode* fromNode = findIntersection(from);
            IntersectionNode* toNode = findIntersection(to);
            removeTravelTime(fromNode, to);
            removeTravelTime(toNode, from);
        }
        void print()
        {
            IntersectionNode* temp = head;
            while(temp != NULL)
            {
                cout << temp->name << " -> ";
                Travel_time* temp2 = temp->edge;
                while(temp2 != NULL)
                {
                    cout << " ("  << temp2->destination<<" , " << temp2->time << ")  ";
                    temp2 = temp2->next;
                }
                cout << endl;
                temp = temp->next;
            }
        }
};
void read_roadNetwork(const string& filename, RoadNetwork& road) 
{
    ifstream file(filename);
    if (!file.is_open()) 
    {
        cout << "Error: Unable to open the file" << endl;
        return;
    }

    string line;
    getline(file, line); // Skip the first line

    while (getline(file, line)) 
    {
        // Find the positions of the delimiters
        int pos1 = line.find(',');
        int pos2 = line.find_last_of(',');

        // Validate positions
        if (pos1 == -1 || pos2 == -1 || pos1 == pos2) 
        {
            cout << "Error: Invalid line format: " << line << endl;
            continue;
        }

        // Extract fields
        string from = line.substr(0, pos1);
        string to = line.substr(pos1 + 1, pos2 - pos1 - 1);
        string weightStr = line.substr(pos2 + 1);

        // Check if weightStr is a valid number
        bool isValidNumber = true;
        for (char c : weightStr) 
        {
            if (!isdigit(c)) 
            {
                isValidNumber = false;
                break;
            }
        }

        if (isValidNumber) 
        {
            int weight = stoi(weightStr); // Convert the weight to an integer
            road.addTravelTime(from, to, weight);
        } 
        else 
        {
            cout << "Error: Invalid weight value in line: " << line << endl;
        }
    }

    file.close();
}

int main()
{
    RoadNetwork road;
    read_roadNetwork("road_network.csv", road);
    road.print();
    return 0;    
}
